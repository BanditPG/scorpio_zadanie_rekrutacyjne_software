import { useEffect, useState, useRef } from "react";
import {useRosHook} from "../contexts/RosConnectionContext";
import ROSLIB from "roslib";
import { motorSettings } from "../settings/rosSettings";

export default function useMotor(motorIndex) {
  const MAX_STRENGTH = motorSettings.maxStrength;
  const MIN_STRENGTH = motorSettings.minStrength;

  const { rosRef, rosStatus } = useRosHook();

  const [position, setPosition] = useState(0);
  const [strength, setStrength] = useState(100);

  const controlTopic = useRef(null);

  useEffect(() => {
    if (rosStatus.status !== "connected")
      return

    let motorDataTopic = subscribeToMotorPosition()
    initMotorControlTopic()

    motorDataTopic.subscribe((message) => {
      setPosition(message.data)
    })

    return () => {
      stop()
      motorDataTopic.unsubscribe()
      controlTopic.current = null
    }
  }, [motorIndex, rosStatus]);

  function subscribeToMotorPosition() {
    let motorDataTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: motorSettings[motorIndex].positionTopic.name,
      messageType: motorSettings[motorIndex].positionTopic.messageType
    })

    motorDataTopic.subscribe((message) => {
      setPosition(message.data)
    })

    return motorDataTopic
  }

  function initMotorControlTopic() {
    let motorControlTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: motorSettings[motorIndex].controlTopic.name,
      messageType: motorSettings[motorIndex].controlTopic.messageType
    })

    motorControlTopic.publish({ data: 0 })
    controlTopic.current = {
      ref: motorControlTopic,
      lastSet: 0
    }
  }
  
  function turnLeft() {
    if (controlTopic.current === null || controlTopic.current.lastSet === -strength)
      return

    controlTopic.current.ref.publish({ data: -strength })
    controlTopic.current.lastSet = -strength
  }

  function turnRight() {
    if (controlTopic.current === null || controlTopic.current.lastSet === strength)
      return

    console.log(`Turning right: ${strength}`)
    controlTopic.current.ref.publish({ data: strength })
    controlTopic.current.lastSet = strength
  }

  function stop() {
    if (controlTopic.current === null || controlTopic.current.lastSet === 0)
      return

    controlTopic.current.ref.publish({ data: 0 })
    controlTopic.current.lastSet = 0
  }

  function setStrengthClamp(value) {
    if (value > MAX_STRENGTH)
      setStrength(MAX_STRENGTH)
    else if (value < MIN_STRENGTH)
      setStrength(MIN_STRENGTH)
    else
      setStrength(value)
  }

  return {
    position,
    strength,
    setStrength: setStrengthClamp,
    turnLeft,
    turnRight,
    stop
  }
}
