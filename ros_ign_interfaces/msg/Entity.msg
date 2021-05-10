# Entity type: constant definition
uint8 NONE      = 0
uint8 LIGHT     = 1
uint8 MODEL     = 2
uint8 LINK      = 3
uint8 VISUAL    = 4
uint8 COLLISION = 5
uint8 SENSOR    = 6
uint8 JOINT     = 7

uint64 id      # Entity unique identifier accross all types. Defaults to 0
string name    # Entity name, which is not guaranteed to be unique.
uint8 type     # Entity type.
