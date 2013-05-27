from comptests.registrar import comptests_for_all
from rosstream2boot import (get_conftools_robot_adapters, get_conftools_explogs,
    get_conftools_cmd_adapters, get_conftools_obs_adapters)



for_all_robot_adapters = comptests_for_all(get_conftools_robot_adapters())
for_all_obs_adapters = comptests_for_all(get_conftools_obs_adapters())
for_all_cmd_adapters = comptests_for_all(get_conftools_cmd_adapters())
for_all_explogs = comptests_for_all(get_conftools_explogs())

print('loaded tests')


@for_all_robot_adapters
def check_robot_adapter(id_ob, ob):
    pass

@for_all_obs_adapters
def check_obs_adapter(id_ob, ob):
    pass

@for_all_cmd_adapters
def check_cmd_adapter(id_ob, ob):
    pass

@for_all_explogs
def check_explog(id_ob, ob):
    pass
