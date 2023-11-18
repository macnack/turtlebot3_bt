#!/usr/bin/env python3
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import behaviours
from py_trees_ros import subscribers as py_trees_subscribers
import sensor_msgs.msg as sensor_msgs


def create_root() -> py_trees.behaviour.Behaviour:
    # Kompozycja typu Parallel, będzie korzeniem naszego drzewa.
    # Wszyskie połączone do niego dzieci będą wykonywane równocześnie
    root = py_trees.composites.Parallel(
        name="Turtlebot3 Behaviour Tree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=True
        )
    )

    # zachowanie Rotate, powoduje obrót robota w miejscu.
    rotation = behaviours.Rotate(name="RotateRobot")

    # Utworzenie zachowania Drive (należy zaimplementować), które wykonuje ruch do przodu.
    driving = behaviours.Drive(name="DriveRobot")

    # zachowanie subskrybujące skan lasera z robota i przekazujące
    # go do Tablicy (przekazuje wyłącznie pole "ranges")
    laser2BB = py_trees_subscribers.ToBlackboard(name="Laser2BB",
                                                 topic_name="/scan",
                                                 topic_type=sensor_msgs.LaserScan,
                                                 blackboard_variables={
                                                     '/scan': 'ranges'},
                                                 qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                                                 clearing_policy=py_trees.common.ClearingPolicy.NEVER)

    # funkcja sprawdzająca czy odległość do przeszkody jest większa niż 3 metry
    # musi odczytać wartości skanera zapisane w Tablicy
    # (tablica jest domyślnie przekazywana przez EternalGuard)
    def check_distance_in_front(blackboard: py_trees.blackboard.Blackboard) -> bool:
        DISTANCE_THRESHOLD = 3
        scan = blackboard.scan
        # take only +- 10 degrees from the front
        scan = scan[0:10] + scan[-10:]
        # check if in the scan there is a distance larger than 3 meters
        for distance in scan:
            if distance > DISTANCE_THRESHOLD:
                return True
        print(scan)
        # TODO: Sprawdzanie czy dystans jest większy niż 3 metry.
        return False

    # Dodanie dekoratora EternalGuard z warunkiem dystansu do przeszkody do jazdy na wprost.
    distance_check = py_trees.decorators.EternalGuard(
        name="DistanceCheck",
        child=driving,
        condition=check_distance_in_front,
        blackboard_keys={"scan"}
    )
    failure_selector = py_trees.composites.Selector(name="FailureSelector", memory=False)
    failure_selector.add_child(py_trees.behaviours.Failure(name="Failure"))
    failure_selector.add_child(distance_check)
    failure_selector.add_child(rotation)
    # Modyfikacja drzewa tak, żeby było możliwe realizowanie założeń z instrukcji.
    root.add_child(laser2BB)
    # root.add_child(rotation)
    root.add_child(failure_selector)

    # Zwracanie korzenia i połączonych do niego dzieci
    return root


def main():
    rclpy.init()
    root = create_root()

    # tworzenie drzewa z korzenia (istotne jest użycie klasy z rozszerzenia "_ros",
    # aby drzewo było nodem)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    # metoda tick_tock powoduje cykliczne uruchamianie drzewa z przerwą między wywołaniami.
    # możliwe jest przekazanie parametru "number_of_iterations" aby określić ile razy ma być wywołane
    # dostępna jest również metoda tick, wykonująca pojedyncze wywołanie drzewa
    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()