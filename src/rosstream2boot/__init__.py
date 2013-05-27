""" Functions for converting to/from ROS messages """

import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from .configuration import * 
from .interfaces import *
from .library import *
from .programs import *



def get_comptests():
    from . import unittests
    from comptests import get_comptests_app
    app = get_comptests_app(get_rs2b_config())
    return [app]
