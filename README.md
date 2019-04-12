# pyhexapod

Hexapod simulation in pybullet

## Dependencies (use pip or your favorite package manager)

- python3
- pybullet
- numpy

Clone this repository and run :

```
git submodule update --init --recursive
```

## How to use ?

### Launch a simple simulation

Run :

``` 
python simulator.py
```
It will run a hexapod episode, the controller parameters have been designed by hand.

### Create map elites maps

Run :

``` 
python simulator_map_elites.py
```

### Launch a simple simulation with a damaged hexapod

In simulator.py use the damage = True option from eval_hexapod(ctrl,gui_eval = False,damage = False)
It will load the urdf of the damaged hexapod (missing leg) in the urdf folder.
