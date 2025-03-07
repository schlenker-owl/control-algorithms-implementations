import matplotlib.pyplot as plt
import control as ctrl
import numpy as np

from controllers.imc_controller import IMCController

def main():
    # 1. Define a simple plant: G(s) = 1 / (s+1)
    #    We'll treat the entire G(s) as "invertible" for demonstration.
    real_plant = ctrl.tf([1], [0.8, 1])  # 1/(0.8*s + 1)

    # 2. Nominal (good) portion is Gm(s) = real_plant in this simple example
    #    If you had a more complex plant, you'd factor out the non-invertible parts.
    Gm = real_plant

    # 3. Define a filter F(s). Common IMC filter: F(s) = 1 / (lambda*s + 1)
    #    Let's pick lambda=1 for demonstration
    lambda_val = 1.0
    F = ctrl.tf([1], [lambda_val, 1])

    # 4. Create IMC controller
    imc = IMCController(plant_nominal=Gm, filter_tf=F)

    # 5. Build the IMC closed-loop with the real plant
    T_imc = imc.imc_closed_loop_tf(real_plant)

    # 6. Step response
    t = np.linspace(0, 10, 300)
    t_resp, y_resp = ctrl.step_response(T_imc, t)

    # 7. Compare to a naive feedback approach (maybe a P controller)
    Kp = 1.0
    naive_controller = ctrl.tf([Kp], [1])
    T_naive = ctrl.feedback(ctrl.series(naive_controller, real_plant), 1)

    t_naive, y_naive = ctrl.step_response(T_naive, t)

    # 8. Plot
    plt.figure()
    plt.plot(t_resp, y_resp, label='IMC Controlled')
    plt.plot(t_naive, y_naive, '--', label='Naive P Control')
    plt.xlabel("Time (s)")
    plt.ylabel("Response")
    plt.title("IMC vs. Naive Control")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
