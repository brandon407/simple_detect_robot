"""
Industrial Knowledge Base — RAG-enhanced domain knowledge retrieval.

Loads structured industrial knowledge (standards, SOPs, equipment specs)
and provides semantic search for context-enhanced LLM prompting.

Architecture:
- JSON knowledge base → text chunks → embedding vectors → similarity search
- In-memory mode by default, ChromaDB optional for persistence
"""
import json
import logging
import os
import re
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# Try ChromaDB for persistent vector storage
try:
    import chromadb
    from chromadb.config import Settings
    HAS_CHROMA = True
except ImportError:
    HAS_CHROMA = False

# Try sentence-transformers for embeddings
try:
    from sentence_transformers import SentenceTransformer
    HAS_EMBEDDINGS = True
except ImportError:
    HAS_EMBEDDINGS = False


class IndustrialKnowledgeBase:
    """RAG-powered industrial domain knowledge retrieval."""

    def __init__(self, knowledge_file: Optional[str] = None,
                 db_path: str = '~/.inspection_robot/knowledge'):
        self._documents: list[dict] = []
        self._chunks: list[str] = []
        self._chunk_metadata: list[dict] = []
        self._embeddings: Optional[np.ndarray] = None
        self._embedder: Optional['SentenceTransformer'] = None
        self._chroma_client: Optional['chromadb.Client'] = None
        self._chroma_collection: Optional['chromadb.Collection'] = None

        # Initialize embedder
        if HAS_EMBEDDINGS:
            try:
                self._embedder = SentenceTransformer(
                    'paraphrase-multilingual-MiniLM-L12-v2')
                logger.info('Embedding model loaded')
            except Exception as e:
                logger.warning(f'Embedding model not available: {e}')

        # Initialize ChromaDB (optional)
        if HAS_CHROMA:
            try:
                db_path_abs = os.path.expanduser(db_path)
                os.makedirs(db_path_abs, exist_ok=True)
                self._chroma_client = chromadb.PersistentClient(
                    path=db_path_abs,
                    settings=Settings(anonymized_telemetry=False),
                )
                self._chroma_collection = self._chroma_client.get_or_create_collection(
                    'industrial_knowledge')
                logger.info(f'ChromaDB initialized: {db_path_abs}')
            except Exception as e:
                logger.warning(f'ChromaDB not available: {e}')

        # Load knowledge
        if knowledge_file and os.path.exists(knowledge_file):
            self.load_knowledge(knowledge_file)

    def load_knowledge(self, knowledge_file: str):
        """Load industrial knowledge from JSON file.

        Expected format:
        {
            "industrial_knowledge": [
                {
                    "category": "safety_standards",
                    "title": "...",
                    "content": "..."
                },
                ...
            ]
        }
        """
        try:
            with open(knowledge_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            documents = data.get('industrial_knowledge', [])
            self._documents = documents
            self._chunk_and_index(documents)
            logger.info(
                f'Loaded {len(documents)} knowledge entries '
                f'→ {len(self._chunks)} chunks')
        except Exception as e:
            logger.error(f'Failed to load knowledge: {e}')

    def _chunk_and_index(self, documents: list[dict]):
        """Split documents into chunks and build vector index."""
        self._chunks = []
        self._chunk_metadata = []

        for doc in documents:
            # Split long content into paragraphs
            content = doc.get('content', '')
            paragraphs = [p.strip() for p in content.split('\n') if p.strip()]

            for para in paragraphs:
                # Further split very long paragraphs
                if len(para) > 500:
                    sentences = re.split(r'(?<=[。！？.!?])', para)
                    for sent in sentences:
                        sent = sent.strip()
                        if len(sent) > 10:
                            self._chunks.append(sent)
                            self._chunk_metadata.append({
                                'title': doc.get('title', ''),
                                'category': doc.get('category', ''),
                                'source': doc.get('title', ''),
                            })
                else:
                    self._chunks.append(para)
                    self._chunk_metadata.append({
                        'title': doc.get('title', ''),
                        'category': doc.get('category', ''),
                        'source': doc.get('title', ''),
                    })

        # Build embeddings
        if self._embedder and self._chunks:
            try:
                self._embeddings = self._embedder.encode(
                    self._chunks, convert_to_numpy=True)
                logger.debug(f'Built embeddings: {self._embeddings.shape}')
            except Exception as e:
                logger.warning(f'Embedding build failed: {e}')
                self._embeddings = None

        # Sync to ChromaDB
        if self._chroma_collection:
            try:
                ids = [f'chunk_{i}' for i in range(len(self._chunks))]
                metas = [{**m, 'text': c}
                         for m, c in zip(self._chunk_metadata, self._chunks)]
                # Only add new chunks
                existing = set(self._chroma_collection.get()['ids'])
                new_items = [(id_, chunk, meta)
                             for id_, chunk, meta in zip(ids, self._chunks, metas)
                             if id_ not in existing]
                if new_items:
                    self._chroma_collection.add(
                        ids=[i[0] for i in new_items],
                        documents=[i[1] for i in new_items],
                        metadatas=[i[2] for i in new_items],
                    )
            except Exception as e:
                logger.debug(f'ChromaDB sync skipped: {e}')

    def search(self, query: str, top_k: int = 5) -> list[dict]:
        """Search knowledge base for relevant context.

        Args:
            query: Natural language search query.
            top_k: Number of results to return.

        Returns:
            List of {content, title, category, score} dicts.
        """
        if not self._chunks:
            return []

        results = []

        # Vector similarity search
        if self._embedder is not None and self._embeddings is not None:
            try:
                query_embedding = self._embedder.encode([query], convert_to_numpy=True)
                scores = np.dot(self._embeddings, query_embedding.T).flatten()
                top_indices = np.argsort(scores)[::-1][:top_k]

                for idx in top_indices:
                    if scores[idx] > 0.1:
                        results.append({
                            'content': self._chunks[idx],
                            'score': float(scores[idx]),
                            **self._chunk_metadata[idx],
                        })
            except Exception as e:
                logger.debug(f'Vector search failed: {e}')

        # Fallback: keyword search
        if not results:
            results = self._keyword_search(query, top_k)

        return results

    def _keyword_search(self, query: str, top_k: int) -> list[dict]:
        """Simple keyword-based fallback search."""
        query_terms = set(query.lower().split())
        scored = []

        for i, chunk in enumerate(self._chunks):
            chunk_lower = chunk.lower()
            score = sum(1 for term in query_terms if term in chunk_lower)
            if score > 0:
                scored.append((score, i))

        scored.sort(key=lambda x: x[0], reverse=True)
        results = []
        for score, idx in scored[:top_k]:
            results.append({
                'content': self._chunks[idx],
                'score': float(score),
                **self._chunk_metadata[idx],
            })
        return results

    def build_context(self, query: str, max_chunks: int = 3) -> str:
        """Build a RAG context string for injection into LLM prompts.

        Args:
            query: Search query.
            max_chunks: Max knowledge chunks to include.

        Returns:
            Formatted context string, or empty if nothing found.
        """
        results = self.search(query, top_k=max_chunks)
        if not results:
            return ''

        parts = ['## 相关工业知识 (来自知识库)']
        for i, r in enumerate(results):
            parts.append(
                f"### {r.get('title', '条目' + str(i + 1))}\n"
                f"分类: {r.get('category', '通用')}\n"
                f"内容: {r['content']}\n"
                f"相关度: {r['score']:.2f}"
            )
        return '\n\n'.join(parts)

    def list_categories(self) -> list[str]:
        """List all knowledge categories."""
        cats = set()
        for doc in self._documents:
            cat = doc.get('category', '')
            if cat:
                cats.add(cat)
        return sorted(cats)

    def get_stats(self) -> dict:
        """Return knowledge base statistics."""
        return {
            'documents': len(self._documents),
            'chunks': len(self._chunks),
            'has_embeddings': self._embeddings is not None,
            'has_chroma': self._chroma_collection is not None,
            'categories': self.list_categories(),
        }
