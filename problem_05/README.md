Run the Cartpole.ipynb with python3. 
Included packages

gym==0.21.0
numpy 

*note: we needed a older version of gym to run the simulation.

The change for being 50% more aggresive to the right can be found in the following line of code in cartpole_env.py:

```
        #######################################################################
        #Make force to the right 50% more aggressive
        if action == 1:
            force = force * 1.5
        #######################################################################
```



Run the frozen_lake.ipynb with python3.

Included packages

gym==0.21.0
numpy

*note: we needed a older version of gym to run the simulation.

The change to use a randomly generated enviroment can be found in the following line of code in frozen_lake_env.py on line 16:

```
        #######################################################################
        #Use a randomly generated enviroment
        self.desc = generate_random_map(size=self.nrow, p=self.p)
        #######################################################################
```