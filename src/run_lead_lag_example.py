import matplotlib.pyplot as plt
import control as ctrl

from controllers.lead_lag_controller import lead_compensator
from data.synthetic_data import SyntheticDataGenerator

def main():
    # 1. Define a simple plant G(s)
    #    E.g.: G(s) = 1 / (s+1)
    G = ctrl.tf([1], [1, 1])

    # 2. Create lead compensator
    #    Suppose we have K=2, tau=1, alpha=0.2
    lead = lead_compensator(K=2, tau=1, alpha=0.2)

    # 3. Closed-loop transfer function
    closed_loop = ctrl.feedback(lead * G, 1)

    # 4. Step response
    t, y = ctrl.step_response(closed_loop, T=10)

    # 5. Plot
    plt.figure()
    plt.plot(t, y, label='Output')
    plt.title('Lead Compensator Closed-Loop Step Response')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
