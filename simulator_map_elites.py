from simulator import *
from py_map_elites.map_elites import *
p1 = \
	    {
	        "cvt_samples": 400000,
	        "batch_size": 200,
	        "random_init": 1000,
		"random_init_batch": 500,
	        "sigma_iso": 0.01,
	        "sigma_line": 0.2,
	        "dump_period": 5,
	        "parallel": True,
	        "cvt_use_cache": True,
		"min": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	        "max": [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
	    }
archive = compute(6,36, eval_hexapod, n_niches=40000, n_gen=20000,params = p1)
