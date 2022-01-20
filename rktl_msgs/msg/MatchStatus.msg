Header header

int8 PAUSED=0           # before the match is started
int8 ONGOING=1          # match is currently going on
int8 STOPPED=2          # match ends with no winning team
int8 VICTORY_TEAM_A=3   # match ends with team A winning
int8 VICTORY_TEAM_B=4   # match ends with team B winning

# status of the match using above constants
int8 status