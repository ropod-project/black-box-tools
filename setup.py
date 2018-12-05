from setuptools import setup, find_packages

setup(name='black_box_tools',
      version='1.0.0',
      description='A library for supporting the offline processing of data from a robotic black box',
      url='https://github.com/ropod-project/black-box-tools',
      author='Alex Mitrevski',
      author_email='aleksandar.mitrevski@h-brs.de',
      keywords='black_box fault_detection robotics',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      project_urls={
          'Source': 'https://github.com/ropod-project/black-box-tools'
      })
