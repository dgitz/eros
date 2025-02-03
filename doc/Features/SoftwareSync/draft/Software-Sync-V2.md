# **_DRAFT DRAFT DRAFT_**

# Architecture
* Class that determines architecture.
* Local and remote modes

# Sync Options
* dry run
* load file
  * specify target (or all)
  * Device with child devices
* User name
* Hostname


# Functionality
* Config, Source, Binary files
* Mode:auto
  * If arch matches local, sync config and binary. 
  * If not, sync config and source
* Config file is optional, specify everything in commands with default parameters.
* All file/folders to be transferred are put in one big list, which is then looped on to actually transfer.
* File config mode just calls manual mode functions, so thats always tested.
* Transfer mode:
  * Push (now)
  * Pull (future)

# Target Actions
* Sync to Child
* Build
* Launch (NO, not part of this process)

# Other
* Better class design
* Doxygen comments