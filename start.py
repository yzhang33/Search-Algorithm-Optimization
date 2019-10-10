import racetrack as rt
import sample_probs as sp
import time
import sample_heuristics as sh
import proj1 as p1

start_time = time.time()
rt.main(sp.lhook16, "a*", p1.h_proj1, p1, draw=1, verbose=1,title='simple')
end_time = time.time()
print("------%s sec--------"%(end_time-start_time))
#p1.h_proj1