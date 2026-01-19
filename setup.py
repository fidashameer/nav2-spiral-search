from setuptools import setup

package_name = 'nav2_spiral_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fida Shameer',
    maintainer_email='your@email.com',
    description='Nav2-based spiral search and cone approach using waypoint goals',
    license='MIT',
    entry_points={
        'console_scripts': [
            'spiral_search_node = nav2_spiral_search.spiral_search_node:main',
            'cone_detection_pose_node = nav2_spiral_search.cone_detection_pose_node:main',
            'cone_approach_node = nav2_spiral_search.cone_approach_node:main',
        ],
    },
)
