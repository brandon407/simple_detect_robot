from setuptools import find_packages, setup

package_name = 'llm_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/llm_agent.launch.py']),
        ('share/' + package_name + '/config',
            ['config/llm_config.yaml',
             'config/knowledge_base.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brandon407',
    maintainer_email='brandon407@github.com',
    description='LLM agent for industrial inspection Q&A',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = llm_agent.llm_node:main',
        ],
    },
)
