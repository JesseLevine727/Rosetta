# Place-and-route constraints: a tightened clock target so the optimiser works
# for the slow corner (see PLAN.md, hardening recipe). Sign-off uses signoff.sdc.
set ::env(CLOCK_PERIOD) 6.0
source $::env(FALLBACK_SDC)
