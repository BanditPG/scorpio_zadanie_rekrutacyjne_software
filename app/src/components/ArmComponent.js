import { useEffect, useRef, useState } from 'react'
import useMotorServices from '../hooks/useMotorServices'
import { useRosHook } from '../contexts/RosConnectionContext'
import { useMotor } from '../contexts/RosMotorsContext'

export default function ArmComponent() {
  const { rosStatus } = useRosHook()
  const { getMotorJointsLenght } = useMotorServices()

  const { position: position1, home: home1 } = useMotor(0)
  const { position: position2, home: home2 } = useMotor(1)
  const { position: position3, home: home3 } = useMotor(2)

  const [jointsLength, setJointsLength] = useState([0, 0, 0])
  const canvasRef = useRef(null)

  const scale = 0.7
  const scaledJointsLength = jointsLength.map(
    jointLength => jointLength * scale
  )

  useEffect(() => {
    if (rosStatus.status === 'connected') {
      ;(async () => {
        setJointsLength(await getMotorJointsLenght())
      })()
    }
  }, [rosStatus])

  useEffect(() => {
    if (
      jointsLength[0] === 0 ||
      jointsLength[1] === 0 ||
      jointsLength[2] === 0
    ) {
      return
    }

    const ctx = canvasRef.current.getContext('2d')

    const { width, height } = canvasRef.current.getBoundingClientRect()

    ctx.clearRect(0, 0, width, height)
    ctx.strokeStyle = 'white'
    ctx.fillStyle = 'white'

    let direction = directionVector(position1)
    let vector = scaleVector(direction, scaledJointsLength[0])
    let endPosition1 = moveVector([width / 2, height / 2], vector)

    drawCirlce(ctx, width / 2, height / 2)
    drawLine(ctx, width / 2, height / 2, endPosition1[0], endPosition1[1])

    direction = directionVector(position2)
    vector = scaleVector(direction, scaledJointsLength[1])
    let endPosition2 = moveVector(endPosition1, vector)

    drawCirlce(ctx, endPosition1[0], endPosition1[1])
    drawLine(
      ctx,
      endPosition1[0],
      endPosition1[1],
      endPosition2[0],
      endPosition2[1]
    )

    direction = directionVector(position3)
    vector = scaleVector(direction, scaledJointsLength[2])
    let endPosition3 = moveVector(endPosition2, vector)

    drawCirlce(ctx, endPosition2[0], endPosition2[1])
    drawLine(
      ctx,
      endPosition2[0],
      endPosition2[1],
      endPosition3[0],
      endPosition3[1]
    )
  }, [position1, position2, position3])

  function drawCirlce(ctx, x, y) {
    ctx.beginPath()
    ctx.arc(x, y, 15, 0, 2 * Math.PI)
    ctx.fill()
  }

  function drawLine(ctx, x1, y1, x2, y2) {
    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x2, y2)
    ctx.stroke()
  }

  function directionVector(position) {
    const step = (2 * Math.PI) / 4095
    const angle = position * step

    return [Math.cos(angle), Math.sin(angle)]
  }

  function scaleVector(vector, scale) {
    return [vector[0] * scale, vector[1] * scale]
  }

  function moveVector(vector, directionVector) {
    return [vector[0] + directionVector[0], vector[1] + directionVector[1]]
  }

  return (
    <div className="arm-component">
      <div>
        <canvas
          width={1200 * scale + 'px'}
          height={1200 * scale + 'px'}
          ref={canvasRef}></canvas>
      </div>

      <div style={{ flexGrow: '1', textAlign: 'center' }}>
        <button
          onClick={() => {
            home1()
            home2()
            home3()
          }}>
          Home
        </button>

        <JointsLengthTable jointsLength={jointsLength} />
      </div>
    </div>
  )
}

function JointsLengthTable({ jointsLength }) {
  return (
    <table className="arm-joints-length-table">
      <tbody>
        <tr>
          <td>Joint 1</td>
          <td>{jointsLength[0]}</td>
        </tr>
        <tr>
          <td>Joint 2</td>
          <td>{jointsLength[1]}</td>
        </tr>
        <tr>
          <td>Joint 3</td>
          <td>{jointsLength[2]}</td>
        </tr>
      </tbody>
    </table>
  )
}
