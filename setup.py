from setuptools import find_packages, setup
from xml.etree import ElementTree

package_name = 'eng612_wall_following'

metadata = ElementTree.parse('package.xml').getroot()
maintainer = metadata.find('maintainer')

setup(
    name=package_name,
    version=metadata.findtext('version', '0.0.0').strip(),
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=(maintainer.text or '').strip()
        if maintainer is not None else '',
    maintainer_email=maintainer.get('email', '')
        if maintainer is not None else '',
    description=metadata.findtext('description', '').strip(),
    license=metadata.findtext('license', '').strip(),
    entry_points={
        'console_scripts': [
            'scan_monitor = eng612_wall_following.scan_monitor:main',
            'left_wall_follow = eng612_wall_following.left_wall_follow:main',
            'dual_wall_follow = eng612_wall_following.dual_wall_follow:main',
            'pid_speed_optimizer = eng612_wall_following.pid_speed_optimizer:main',
        ],
    },
)
