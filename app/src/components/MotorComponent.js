import { useEffect, useState } from "react";
import useMotor from "../hooks/UseMotor";
import keyboardSettings from "../settings/keyboardSettings";

export default function MotorComponent({ index, isMotorControlEnable }) {
  const {position, strength, setStrength, turnLeft, turnRight, stop} = useMotor(index)
  const [leftKeyPressed, setLeftKeyPressed] = useState(false)
  const [rightKeyPressed, setRightKeyPressed] = useState(false)

  const leftKey = keyboardSettings.motors[index].leftKey
  const rightKey = keyboardSettings.motors[index].rightKey

  useEffect(() => {
    let keyDownHandler = (event) => {
      if (event.key === leftKey) {
        setLeftKeyPressed(true)
        turnLeft()
      } else if (event.key === rightKey) {
        setRightKeyPressed(true)
        turnRight()
      }
    }

    let keyUpHandler = (event) => {
      if (event.key === leftKey) {
        stop()
        setLeftKeyPressed(false)
      } else if (event.key === rightKey) {
        stop()
        setRightKeyPressed(false)
      }
    }

    if (isMotorControlEnable) {
      document.addEventListener("keydown", keyDownHandler)
      document.addEventListener("keyup", keyUpHandler)
    } else {
      stop()
    }

    return () => {
      document.removeEventListener("keydown", keyDownHandler)
      document.removeEventListener("keyup", keyUpHandler)
    }
  }, [isMotorControlEnable, strength]);

  function updateStrength(value) {
    const parsed = parseInt(value)
    if (isNaN(parsed)) 
      return

    setStrength(parsed)
  }

  return (
    <div className="motor-component">
      <p>Motor {index}</p>
      <div style={{ width: "300px", height: "300px", display: "flex", justifyContent: "center", alignItems: "center", border: "1px black dashed" }}>
        <div>Canvas</div>
      </div>
      <div>
        <p>Position: {Math.round((position / 4095) * 360)} degrees</p>
        <div style={{display: "flex", gap: "7px"}}>
          <div className={"keyboard-key" + (leftKeyPressed ? " keyboard-key-pressed" : "")}>{leftKey}</div>
          <div className={"keyboard-key" + (rightKeyPressed ? " keyboard-key-pressed" : "")}>{rightKey}</div>
        </div>
        Strength: <input type="range" min="13" max="100" value={strength} onChange={e => updateStrength(e.target.value)} /> {strength}%
      </div>
    </div>
  );
}
