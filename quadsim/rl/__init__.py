"""
quadsim.rl
==========
Aprendizaje por refuerzo OFFLINE para el lazo de posicion/trayectoria.

Criterio de diseno: no entrenar un RL en una planta lenta. Por eso el
flujo es:

  1. Identificacion (`surrogate.py`): se mide en el simulador 6-DOF completo
     la respuesta  a_des -> aceleracion real  (lazo de actitud + empuje) y se
     ajusta un modelo retardo + primer orden.
  2. Entorno rapido vectorizado (`env.py`): solo la dinamica traslacional con
     ese modelo, arrastre, viento y ALEATORIZACION DE DOMINIO. Corre ~10^5-10^6
     pasos/s en numpy, contra ~1.3x10^3 pasos externos/s del simulador completo.
  3. Entrenamiento PPO en numpy (`ppo.py`, experiments/train_rl.py).
  4. Sim2Real (aqui Sim2Sim): la politica se ejecuta SIN reentrenar en el
     simulador 6-DOF completo (sensores, estimadores, lazo de actitud).
  5. Validacion (experiments/train_rl.py validate).

La inferencia (`policy.py`) es solo numpy: corre en la instalacion de
Windows del usuario, que no tiene torch.
"""
