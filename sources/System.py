import numpy as np
import control as ctr

class System:
    """
    Helper class to simulate linear discrete time dynamical systems in state-space form.
    Initiate with system matrices: A,B,C,D such that:
    x_next = A@x + B@u
    y      = C@x + D@u
    Passing D is optional.
    Passing x0 is optional (will results to all zero per default).
    Run a simulation step with make_step method.
    Reset with reset method.
    Query stored results with methods u, y, x.
    """
    def __init__(self,x_r,K,aux,A,B,C,D=None, x0=None, dt=0.04):
        self.A = A
        self.B = B
        self.C = C

        self.n_x = A.shape[0]
        self.n_u = B.shape[1]
        self.n_y = C.shape[0]

        if D is None:
            D = np.zeros((self.n_y, self.n_u))

        self.D = D

        if x0 is None:
            x0 = np.zeros((self.n_x,1))

        self.x0 = x0
        self.xr = x_r
        self.K = K
        self.aux = aux
        self._x = []
        self._u = []
        self._y = []

        self.dt = dt
        self.t_now = 0
        self._time = []
        


    def make_step(self,u):
        """
        Run a simulation step by passing the current input.
        Returns the current measurement y.
        """

        self._x.append(self.x0)
        self._u.append(u)
        y = self.C@self.x0+self.D@u
        
        y = np.array([y[0], y[1], y[2], y[6], y[7]])
        
        self._y.append(y)
        self._time.append(self.t_now)

        if self.aux == 1:    
            self.x0 = self.A@self.x0  + self.B@self.K@self.xr + self.B@u
        else:
            self.x0 = self.A@self.x0  + self.B@u

        self.t_now += self.dt

        return y

    def reset(self, x0=None):
        if x0 is not None:
            self.x0 = x0
        else:
            self.x0 = np.zeros((8,1))

        self._x = []
        self._u = []
        self._y = []
        self._time = []
        self.t_now = 0

    @property
    def x(self):
        return np.concatenate(self._x,axis=1).T

    @x.setter
    def x(self, *args):
        raise Exception('Cannot set x directly.')

    @property
    def u(self):
        return np.concatenate(self._u,axis=1).T

    @u.setter
    def u(self, *args):
        raise Exception('Cannot set u directly.')

    @property
    def y(self):
        return np.concatenate(self._y,axis=1).T

    @y.setter
    def y(self, *args):
        raise Exception('Cannot set y directly.')

    @property
    def time(self):
        return np.array(self._time)

    @time.setter
    def time(self, *args):
        raise Exception('Cannot set time directly.')




def def_system(m,aux):
    
    aux_1 = np.array([[0, -0.007848],[0.007848, 0],[0, 0]])
    
    aux_2 = np.array([[0, -0.3924],[0.3924, 0],[0, 0]])   
    
    aux_3 = np.array([[0, 0, -1.0464e-4],[0, 1.0464e-4, 0],[-2.8571e-2, 0, 0]])
    
    aux_4 = np.array([[0, 0, -7.848e-3],[0, 7.848e-3, 0],[-1.4286, 0, 0]])
    
       
    A1 = np.hstack((np.identity(3), 0.04*np.identity(3), aux_1))
    A2 = np.hstack((np.zeros((3,3)), np.identity(3), aux_2))
    A3 = np.hstack((np.zeros((2,3)), np.zeros((2,3)), np.identity(2)))
    
    A = np.vstack((A1,A2,A3))   
    
    
    B3 = np.hstack((np.zeros((2,1)), 0.04*np.identity(2)))
    
    B = np.vstack((aux_3,aux_4,B3))

    C = np.identity(8) 
    
    Q = 10*np.identity(8)
    R = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

    K_lqr= ctr.dlqr(A,B,Q,R)[0]
    
    if aux == 1:
        A = A - np.matmul(B,K_lqr)
     

    return A, B, C, K_lqr