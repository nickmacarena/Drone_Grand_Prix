"""Entry point. Connect, arm, run the control loop until done."""

import time

from setup import setup_components


SIM_SERVER_UDP_IP = "0.0.0.0"  # bind to all interfaces — sim sends from VM
SIM_SERVER_UDP_PORT = 14550


def main():
    system_boot_ms = int(time.time() * 1000)
    components = setup_components(SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT, system_boot_ms)

    controller = components["controller"]
    shared = components["shared"]

    print("Arming...", flush=True)
    controller.arm()

    print("Running control loop. Ctrl+C to stop.", flush=True)
    try:
        while True:
            controller.update()
            if shared.race_status and shared.race_status.race_finished:
                print("Race finished!", flush=True)
                break
    except KeyboardInterrupt:
        print("Interrupted.", flush=True)

    # Join background threads
    for name in ("mavlink_rx", "timesync"):
        components[name].get_thread_for_join().join(timeout=1.0)

    print("Exited.", flush=True)


if __name__ == "__main__":
    main()
