import pathlib
import sys
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "competition_mission"))

from competition_mission.mission_state_machine import (
    ArmOutcome,
    ArmResult,
    ArmStation,
    ArmTaskType,
    CheckpointReady,
    CommandType,
    CompetitionMissionStateMachine,
    FlagDetected,
    LightObservation,
    LightSample,
    MarkerPassed,
    MissionConfig,
    MissionState,
    StableLight,
    Tick,
)
from competition_mission.mission_config import mission_config_from_dict


class CompetitionMissionStateMachineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = CompetitionMissionStateMachine(
            MissionConfig(
                traffic_no_result_timeout_s=15.0,
                pickup_timeout_s=30.0,
                drop_timeout_s=20.0,
                pickup_max_attempts=3,
                drop_max_attempts=2,
            )
        )

    @staticmethod
    def command(decision, command_type: CommandType):
        return next(
            command
            for command in decision.commands
            if command.command_type == command_type
        )

    def start_and_reach_traffic_stop(self) -> None:
        decision = self.machine.handle(FlagDetected(now_s=1.0))
        self.assertEqual(decision.state, MissionState.RUN_TO_TRAFFIC_STOP)
        route = self.command(decision, CommandType.SET_ROUTE_ENABLED)
        self.assertTrue(route.enabled)

        decision = self.machine.handle(
            MarkerPassed(now_s=2.0, marker_ref="traffic_light_vision_on")
        )
        light = self.command(decision, CommandType.SET_TRAFFIC_LIGHT_ENABLED)
        self.assertTrue(light.enabled)
        self.assertFalse(
            any(
                command.command_type
                == CommandType.SET_TRAFFIC_STOP_ENABLED
                for command in decision.commands
            )
        )

        decision = self.machine.handle(
            CheckpointReady(now_s=3.0, checkpoint_ref="traffic_light_stop_line")
        )
        self.assertEqual(decision.state, MissionState.WAIT_TRAFFIC_LIGHT)
        stop = self.command(decision, CommandType.SET_TRAFFIC_STOP_ENABLED)
        self.assertTrue(stop.enabled)

    def release_from_traffic(self, now_s: float = 4.0) -> None:
        decision = self.machine.handle(
            StableLight(now_s=now_s, light=LightObservation.GREEN)
        )
        self.assertEqual(decision.state, MissionState.RUN_TO_PICKUP_FRONT)
        release = self.command(decision, CommandType.RELEASE_TO_CHECKPOINT)
        self.assertEqual(release.checkpoint_ref, "pickup_front")
        light = self.command(decision, CommandType.SET_TRAFFIC_LIGHT_ENABLED)
        self.assertFalse(light.enabled)
        stop = self.command(decision, CommandType.SET_TRAFFIC_STOP_ENABLED)
        self.assertFalse(stop.enabled)

    def start_arm_at(self, checkpoint_ref: str, now_s: float):
        decision = self.machine.handle(
            CheckpointReady(now_s=now_s, checkpoint_ref=checkpoint_ref)
        )
        command = self.command(decision, CommandType.START_ARM_TASK)
        self.assertIsNotNone(command.arm_task)
        return decision, command.arm_task

    def test_initial_state_waits_for_flag_then_arms_route(self) -> None:
        self.assertEqual(self.machine.snapshot.state, MissionState.WAIT_START_FLAG)

        decision = self.machine.handle(FlagDetected(now_s=1.0))

        self.assertEqual(decision.state, MissionState.RUN_TO_TRAFFIC_STOP)
        command = self.command(decision, CommandType.SET_ROUTE_ENABLED)
        self.assertTrue(command.enabled)

    def test_repository_mission_config_loads(self) -> None:
        config_path = (
            REPO_ROOT
            / "config"
            / "mission"
            / "indoor_competition_mission.yaml"
        )
        with config_path.open("r", encoding="utf-8") as stream:
            config = mission_config_from_dict(yaml.safe_load(stream))

        self.assertEqual(config.traffic_no_result_timeout_s, 15.0)
        self.assertEqual(config.pickup_max_attempts, 1)
        self.assertEqual(config.pickup_timeout_s, 180.0)
        self.assertEqual(config.pickup_front_ref, "pickup_front")

    def test_marker_enables_light_without_changing_drive_state(self) -> None:
        self.machine.handle(FlagDetected(now_s=1.0))

        decision = self.machine.handle(
            MarkerPassed(now_s=2.0, marker_ref="traffic_light_vision_on")
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_TRAFFIC_STOP)
        self.assertTrue(
            self.command(
                decision, CommandType.SET_TRAFFIC_LIGHT_ENABLED
            ).enabled
        )
        self.assertFalse(
            any(
                command.command_type
                == CommandType.SET_TRAFFIC_STOP_ENABLED
                for command in decision.commands
            )
        )

    def test_stop_checkpoint_enables_vision_and_stop_if_marker_was_missed(self) -> None:
        self.machine.handle(FlagDetected(now_s=1.0))

        decision = self.machine.handle(
            CheckpointReady(now_s=3.0, checkpoint_ref="traffic_light_stop_line")
        )

        self.assertEqual(decision.state, MissionState.WAIT_TRAFFIC_LIGHT)
        self.assertTrue(
            self.command(
                decision, CommandType.SET_TRAFFIC_LIGHT_ENABLED
            ).enabled
        )
        self.assertTrue(
            self.command(decision, CommandType.SET_TRAFFIC_STOP_ENABLED).enabled
        )

    def test_front_pickup_and_front_drop_success_skip_both_rear_stops(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, pickup = self.start_arm_at("pickup_front", 5.0)

        decision = self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=pickup.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="green_bottle",
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_DROP_FRONT)
        self.assertTrue(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "drop_front",
        )

        _, drop = self.start_arm_at("drop_front", 7.0)
        decision = self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=drop.task_id,
                outcome=ArmOutcome.SUCCESS,
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_FINISH)
        self.assertFalse(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "finish_park",
        )

    def test_front_pickup_failure_uses_rear_and_reuses_known_target_type(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, front = self.start_arm_at("pickup_front", 5.0)
        decision = self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=front.task_id,
                outcome=ArmOutcome.TARGET_NOT_FOUND,
                target_type="red_bottle",
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_PICKUP_REAR)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "pickup_rear",
        )

        _, rear = self.start_arm_at("pickup_rear", 7.0)
        self.assertEqual(rear.task_type, ArmTaskType.PICKUP)
        self.assertEqual(rear.station, ArmStation.REAR)
        self.assertEqual(rear.target_type_hint, "red_bottle")

    def test_rear_pickup_success_continues_to_front_drop(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, front = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=front.task_id,
                outcome=ArmOutcome.TARGET_NOT_FOUND,
                target_type="red_bottle",
            )
        )
        _, rear = self.start_arm_at("pickup_rear", 7.0)

        decision = self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=rear.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="red_bottle",
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_DROP_FRONT)
        self.assertTrue(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "drop_front",
        )

    def test_pickup_final_failure_bypasses_drop_tasks(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, front = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=front.task_id,
                outcome=ArmOutcome.INSTRUCTION_NOT_FOUND,
            )
        )
        _, rear = self.start_arm_at("pickup_rear", 7.0)

        decision = self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=rear.task_id,
                outcome=ArmOutcome.OPERATION_FAILED,
            )
        )

        self.assertEqual(decision.state, MissionState.BYPASS_DROP_TASKS)
        self.assertFalse(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "finish_park",
        )

        stale_drop = self.machine.handle(
            CheckpointReady(now_s=9.0, checkpoint_ref="drop_front")
        )
        self.assertEqual(stale_drop.state, MissionState.BYPASS_DROP_TASKS)
        self.assertFalse(stale_drop.commands)

    def test_red_or_yellow_resets_continuous_no_result_timeout(self) -> None:
        self.start_and_reach_traffic_stop()
        self.machine.handle(
            LightSample(now_s=10.0, light=LightObservation.RED)
        )

        before_timeout = self.machine.handle(Tick(now_s=24.9))
        self.assertEqual(before_timeout.state, MissionState.WAIT_TRAFFIC_LIGHT)
        self.assertFalse(before_timeout.commands)

        self.machine.handle(
            LightSample(now_s=25.0, light=LightObservation.YELLOW)
        )
        before_second_timeout = self.machine.handle(Tick(now_s=39.9))
        self.assertEqual(
            before_second_timeout.state, MissionState.WAIT_TRAFFIC_LIGHT
        )

        timeout = self.machine.handle(Tick(now_s=40.0))
        self.assertEqual(timeout.state, MissionState.RUN_TO_PICKUP_FRONT)
        self.assertEqual(
            self.command(
                timeout, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "pickup_front",
        )

    def test_unknown_and_off_do_not_reset_no_result_timeout(self) -> None:
        self.start_and_reach_traffic_stop()
        self.machine.handle(
            LightSample(now_s=10.0, light=LightObservation.UNKNOWN)
        )
        self.machine.handle(
            LightSample(now_s=14.0, light=LightObservation.OFF)
        )

        decision = self.machine.handle(Tick(now_s=18.0))

        self.assertEqual(decision.state, MissionState.RUN_TO_PICKUP_FRONT)

    def test_arm_timeout_cancels_front_task_and_releases_rear_checkpoint(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, pickup = self.start_arm_at("pickup_front", 5.0)

        decision = self.machine.handle(Tick(now_s=35.0))

        self.assertEqual(decision.state, MissionState.RUN_TO_PICKUP_REAR)
        cancel = self.command(decision, CommandType.CANCEL_ARM_TASK)
        self.assertEqual(cancel.task_id, pickup.task_id)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "pickup_rear",
        )

    def test_late_arm_result_from_previous_point_is_ignored(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, front = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=front.task_id,
                outcome=ArmOutcome.TARGET_NOT_FOUND,
            )
        )
        _, rear = self.start_arm_at("pickup_rear", 7.0)

        decision = self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=front.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="green_bottle",
            )
        )

        self.assertEqual(decision.state, MissionState.PICKUP_REAR_TASK)
        self.assertEqual(decision.active_arm_task_id, rear.task_id)
        self.assertFalse(decision.has_cargo)
        self.assertFalse(decision.commands)

    def test_drop_rear_failure_records_cargo_but_continues_to_finish(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, pickup = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=pickup.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="blue_bottle",
            )
        )
        _, drop_front = self.start_arm_at("drop_front", 7.0)
        self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=drop_front.task_id,
                outcome=ArmOutcome.OPERATION_FAILED,
            )
        )
        _, drop_rear = self.start_arm_at("drop_rear", 9.0)
        self.assertEqual(drop_rear.task_type, ArmTaskType.DROP)
        self.assertEqual(drop_rear.station, ArmStation.REAR)
        self.assertEqual(drop_rear.target_type_hint, "blue_bottle")

        decision = self.machine.handle(
            ArmResult(
                now_s=10.0,
                task_id=drop_rear.task_id,
                outcome=ArmOutcome.TIMEOUT,
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_FINISH)
        self.assertTrue(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "finish_park",
        )

    def test_drop_rear_success_clears_cargo_and_continues_to_finish(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, pickup = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=pickup.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="blue_bottle",
            )
        )
        _, drop_front = self.start_arm_at("drop_front", 7.0)
        self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=drop_front.task_id,
                outcome=ArmOutcome.OPERATION_FAILED,
            )
        )
        _, drop_rear = self.start_arm_at("drop_rear", 9.0)

        decision = self.machine.handle(
            ArmResult(
                now_s=10.0,
                task_id=drop_rear.task_id,
                outcome=ArmOutcome.SUCCESS,
            )
        )

        self.assertEqual(decision.state, MissionState.RUN_TO_FINISH)
        self.assertFalse(decision.has_cargo)
        self.assertEqual(
            self.command(
                decision, CommandType.RELEASE_TO_CHECKPOINT
            ).checkpoint_ref,
            "finish_park",
        )

    def test_finish_checkpoint_latches_finished_state(self) -> None:
        self.start_and_reach_traffic_stop()
        self.release_from_traffic()
        _, pickup = self.start_arm_at("pickup_front", 5.0)
        self.machine.handle(
            ArmResult(
                now_s=6.0,
                task_id=pickup.task_id,
                outcome=ArmOutcome.SUCCESS,
                target_type="green_bottle",
            )
        )
        _, drop = self.start_arm_at("drop_front", 7.0)
        self.machine.handle(
            ArmResult(
                now_s=8.0,
                task_id=drop.task_id,
                outcome=ArmOutcome.SUCCESS,
            )
        )

        decision = self.machine.handle(
            CheckpointReady(now_s=9.0, checkpoint_ref="finish_park")
        )

        self.assertEqual(decision.state, MissionState.FINISHED)
        self.assertTrue(decision.finished)
        self.assertEqual(
            self.command(decision, CommandType.MISSION_FINISHED).reason,
            "finish_arrived_and_stopped",
        )


if __name__ == "__main__":
    unittest.main()
