from mpcc.pydubins import DubinsPath, LineSegment, CircularSegment, DubinsSolver
from pytest import approx
import numpy as np
import pytest

from test_pydubins import create_test_segments

ls1, cs1, ls2, cs2, cs3 = create_test_segments()

path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

solver = DubinsSolver()

def test_solver_no_perturbation():
    assert solver.solve(path, [False] * (int(path.num_segments()) - 1 )) == approx(path.get_params())

@pytest.mark.repeat(50)
def test_solver_random_perturbation():
    
    original_params = path.get_params()
    

    perturb = np.random.uniform(-1e-2, 1e-2, size=len(original_params))
    perturbed_params = original_params + perturb
    
    path.set_params(perturbed_params)
    solved_params = solver.solve(path, [False] * (int(path.num_segments()) - 1))
    
    assert solved_params == approx(original_params, abs=1e-1)
        
    path.set_params(original_params)

def test_solver_debug():
    
    solver = DubinsSolver(tolerance=1e-6, max_iter=50, debug=2)
    
    original_params = path.get_params()
    perturb = np.random.uniform(-1e2, 1e2, size=len(original_params))
    perturbed_params = original_params + perturb
    
    path.set_params(perturbed_params)
    solved_params = solver.solve(path, [False] * (int(path.num_segments()) - 1))
    assert path.get_constraint_residuals(solved_params) == approx(np.zeros((path.num_segments() - 1 ) * 2), abs=1e-6)
    
    path.set_params(original_params)
    