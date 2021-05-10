bool pause                                  # Paused state.      
bool step                                   #
uint32 multi_step 0                         # Paused after stepping multi_step.
ros_ign_interfaces/WorldReset reset         #
uint32 seed                                 #
builtin_interfaces/Time run_to_sim_time     # A simulation time in the future to run to and 
                                            # then pause.
