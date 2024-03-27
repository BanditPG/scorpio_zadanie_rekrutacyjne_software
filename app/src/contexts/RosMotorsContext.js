import { useEffect, useState, useRef, useContext, createContext } from 'react'
import { useRosHook } from './RosConnectionContext'
import ROSLIB from 'roslib'
import { motorSettings } from '../settings/rosSettings'

const Motor1Context = createContext(null)
const Motor2Context = createContext(null)
const Motor3Context = createContext(null)

export function useMotor(index) {
  return useContext([Motor1Context, Motor2Context, Motor3Context][index])
}

export function RosMotorsProvider({ children }) {
  return (
    <Motor1Context.Provider value={RosMotorHookProvider(0)}>
      <Motor2Context.Provider value={RosMotorHookProvider(1)}>
        <Motor3Context.Provider value={RosMotorHookProvider(2)}>
          {children}
        </Motor3Context.Provider>
      </Motor2Context.Provider>
    </Motor1Context.Provider>
  )
}

function RosMotorHookProvider(motorIndex) {
  const MAX_STRENGTH = motorSettings.maxStrength
  const MIN_STRENGTH = motorSettings.minStrength

  const { rosRef, rosStatus } = useRosHook()

  const [position, setPosition] = useState(0)
  const [strength, setStrength] = useState(100)
  const [isHomeing, setIsHomeing] = useState(false)

  const controlTopic = useRef(null)

  useEffect(() => {
    if (rosStatus.status !== 'connected') return

    let motorDataTopic = subscribeToMotorPosition()
    initMotorControlTopic()

    return () => {
      stop()
      motorDataTopic.unsubscribe()
      controlTopic.current = null
    }
  }, [motorIndex, rosStatus])

  useEffect(() => {
    if (!isHomeing || rosStatus.status !== 'connected') {
      return
    }

    let direction
    let distance
    if (position > 2047) {
      distance = 4095 - position
      direction = 1
    } else {
      distance = position
      direction = -1
    }

    if (distance < 3) {
      setIsHomeing(false)
      setStrength(100)
      controlTopic.current.ref.publish({ data: 0 })
      controlTopic.current.lastSet = 0
    } else {
      let strength = Math.floor(
        (1 - Math.pow(1 - distance / 2047, 3)) *
          (MAX_STRENGTH - MIN_STRENGTH - 2) +
          MIN_STRENGTH +
          2
      )
      setStrength(strength)

      strength = strength * direction
      controlTopic.current.ref.publish({ data: strength })
      controlTopic.current.lastSet = strength
    }
  }, [isHomeing, position, rosStatus])

  function subscribeToMotorPosition() {
    let motorDataTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: motorSettings[motorIndex].positionTopic.name,
      messageType: motorSettings[motorIndex].positionTopic.messageType,
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
      messageType: motorSettings[motorIndex].controlTopic.messageType,
    })

    motorControlTopic.publish({ data: 0 })
    controlTopic.current = {
      ref: motorControlTopic,
      lastSet: 0,
    }
  }

  function turnLeft() {
    if (
      controlTopic.current === null ||
      controlTopic.current.lastSet === -strength
    )
      return

    controlTopic.current.ref.publish({ data: -strength })
    controlTopic.current.lastSet = -strength
  }

  function turnRight() {
    if (
      controlTopic.current === null ||
      controlTopic.current.lastSet === strength
    )
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

  function home() {
    setIsHomeing(true)
  }

  function setStrengthClamp(value) {
    if (value > MAX_STRENGTH) setStrength(MAX_STRENGTH)
    else if (value < MIN_STRENGTH) setStrength(MIN_STRENGTH)
    else setStrength(value)
  }

  if (isHomeing) {
    return {
      position,
      strength,
      setStrength: () => {},
      turnLeft: () => {},
      turnRight: () => {},
      stop: () => {},
      home: () => {},
    }
  }

  return {
    position,
    strength,
    setStrength: setStrengthClamp,
    turnLeft,
    turnRight,
    stop,
    home,
  }
}
