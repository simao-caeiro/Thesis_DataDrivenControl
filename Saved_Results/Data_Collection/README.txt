Nonlinear -> normal
Nonlinear_band... -> alterações na band
Nonlinear_no_circshift -> não alteração dos PRBS entre inputs
Nonlinear_simple_gen -> geração de PRBS de forma incorreta (forma demasiado simples):
	> exc_param.thrust_exc_signal = idinput(exc_param.T,'prbs',[0 1],[-thrust_exc_amp thrust_exc_amp]);
exc_param.rolld_exc_signal = idinput(exc_param.T,'prbs',[0 1],[-rolld_exc_amp rolld_exc_amp]);
exc_param.pitchd_exc_signal = idinput(exc_param.T,'prbs',[0 1],[-pitchd_exc_amp pitchd_exc_amp]);