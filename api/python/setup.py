from setuptools import setup, find_packages

setup(name='clkdirpwm',
      version='1.0', 
      description = 'provides an interface to at90usb clock and direction to pwm generator',
      author = 'Peter Polidoro',
      author_email = 'polidoro@caltech.edu',
      packages=find_packages(),
      entry_points = {'console_scripts': ['clkdirpwm = clkdirpwm:clkdirpwm_main',]}
      )
      
