import numpy as np

def random_u(u0, switch_prob=0.5, u_max=0.05):
    # Hold the current value with switch_prob chance or switch to new random value.
    u_next = (0.5-np.random.rand(3,1))*u_max # New candidate value.
    switch = np.random.rand() >= (1-switch_prob) # switching? 0 or 1.
    u0 = (1-switch)*u0 + switch*u_next # Old or new value.
    return u0


def data_collection(u0, sys, Td, L, aux_noise, sig_w):
    
    U_L = []
    Y_L = []
    
    for k in range(Td):
        x0 = np.random.randn(8,1)
        # x0 = np.zeros((8,1))
        sys.reset(x0)
      
        for k in range(L):
            u0 = random_u(u0)
            sys.make_step(u0)
        
       
        U_L.append(sys.u.reshape(-1,1))
        Y_L.append(sys.y.reshape(-1,1))
    
    U_L = np.concatenate(U_L,axis=1)
    Y_L = np.concatenate(Y_L,axis=1)
    
    if aux_noise == 1:
        # Add noise to Data:
        Y_L = Y_L+np.random.randn(*Y_L.shape)*sig_w
    
    assert np.linalg.matrix_rank(U_L) == U_L.shape[0], "not persistantly exciting."
    
    return U_L, Y_L