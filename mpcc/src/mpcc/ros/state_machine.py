from enum import Enum
from dataclasses import dataclass
from typing import Callable


class SMState(Enum):
    DISCONNECTED = 0
    CONNECTED = 1
    ARMING = 2
    ARMED = 3
    SET_AUTO = 4
    TAKING_OFF = 5
    WP_FOLLOW = 6
    MPCC = 7
    LANDING = 8
    SYSTEM_IDENTIFICATION = 9
    FTS = 254
    TIMEOUT = 255


@dataclass
class StateTransition:
    from_state: SMState
    to_state: SMState
    condition: Callable[[], bool]
    action: Callable[[], None] | None = None
    after: float | None = None
    timeout: float | None = None


class StateMachine:
    def __init__(self, node):
        self.node = node
        self.state = SMState.DISCONNECTED
        self.pending_future = None
        self.timer = None
        self.last_transition_time = self.node.get_clock().now()
        self.transitions = self._define_transitions()

    def _define_transitions(self) -> list[StateTransition]:
        return [
            StateTransition(
                SMState.DISCONNECTED,
                SMState.CONNECTED,
                lambda: self.node.current_state.connected,
            ),
            StateTransition(
                SMState.CONNECTED,
                SMState.DISCONNECTED,
                lambda: not self.node.current_state.connected,
            ),
            StateTransition(
                SMState.CONNECTED,
                SMState.ARMING,
                self.node.arming_ready,
                action=self.node.arm_drone,
                after=1.0,
                timeout=3.0,
            ),
            StateTransition(
                SMState.ARMING,
                SMState.CONNECTED,  # failed to arm
                lambda: self.pending_future
                and self.pending_future.done()
                and not self.pending_future.result().success,
                action=lambda: self.node.get_logger().error("Arming failed"),
            ),
            StateTransition(
                SMState.ARMING,
                SMState.ARMED,
                lambda: self.node.current_state.armed and self._is_ready(),
            ),
            StateTransition(
                SMState.ARMED,
                SMState.CONNECTED,
                lambda: not self.node.arm_ready,
                action=self.node.disarm_drone,
            ),
            StateTransition(
                SMState.ARMED,
                SMState.TAKING_OFF,
                lambda: self.node.current_state.mode != "TAKEOFF",
                action=self.node.set_auto,
                after=1.0,
            ),
            StateTransition(
                SMState.TAKING_OFF,
                SMState.WP_FOLLOW,
                self.node.takeoff_altitude_reached,
                # action=self.node.set_auto,
                after=5.0,
            ),
            StateTransition(
                SMState.SET_AUTO,
                SMState.MPCC,
                self.node.mpcc_ready,
                action=self.node.start_mpcc,
                after=5.0,
            ),
            StateTransition(
                SMState.SET_AUTO,
                SMState.SYSTEM_IDENTIFICATION,
                self.node.system_id_ready,
                action = self.node.start_system_id,
                after = 5.0
            ),
            StateTransition(
                SMState.SYSTEM_IDENTIFICATION,
                SMState.SET_AUTO,
                self.node.system_id_finished,
                action = self.node.finish_system_id,
            ),
            StateTransition(
                SMState.MPCC,
                SMState.SET_AUTO,
                lambda: False, # fallback to onboard auto
                action=self.node.set_auto
            ),
            StateTransition(
                SMState.WP_FOLLOW,
                SMState.LANDING,
                lambda: False
            ),
            StateTransition(
                SMState.MPCC,
                SMState.LANDING,
                lambda: False # done with path
            )
        ]

    def _is_ready(self):
        return self.pending_future is None or self.pending_future.done()

    def _set_timeout(self, timeout):
        self._clear_timeout
        self.timer = self.node.create_timer(timeout, self._on_timeout)

    def _clear_timeout(self):
        if self.timer:
            self.timer.destroy()
        

    def _on_timeout(self):
        self.node.get_logger().info(f"Timeout in state {self.state}")
        self.state = SMState.TIMEOUT
        if self.timer:
            self.timer.destroy()

    def update(self):
        """Call from timer callback"""
        for transition in self.transitions:
            if (
                transition.from_state == self.state
                and transition.condition()
                and (
                    not transition.after
                    or (
                        self.node.get_clock().now() - self.last_transition_time
                    ).nanoseconds
                    > 1e9 * transition.after
                )
            ):
                self.node.get_logger().info(
                    f"Transitioning: {self.state} -> {transition.to_state}"
                )
                self.state = transition.to_state
                self.last_transition_time = self.node.get_clock().now()
                self._clear_timeout()

                if transition.action:
                    self.pending_future = transition.action()
                    if transition.timeout:
                        self._set_timeout(transition.timeout)
                break  # only one transition per tick