import { useState } from "react"
import MotorComponent from "./MotorComponent"
import {useRosHook} from "../contexts/RosConnectionContext"
import StatusComponent from "./StatusComponent"

function App() {
  let {rosStatus} = useRosHook()
  let [isMotorControlEnable, setMotorControlEnable] = useState(false)

  function changeMotorControlStatus() {
    setMotorControlEnable(!isMotorControlEnable)
  }

  return (
    <div className="App">
      <div style={{display: "flex", gap: "15px"}}>
        <StatusComponent text="Connection to ROS" staus={rosStatus.status === "connected"} />
        <StatusComponent text="Motor control" staus={isMotorControlEnable} changeStatus={changeMotorControlStatus} />
      </div>
  
      <div style={{display: "flex", justifyContent: "space-between", marginTop: "50px"}}>
        <MotorComponent index={0} leftKey="q" rightKey="e" isMotorControlEnable={isMotorControlEnable} />
        <MotorComponent index={1} leftKey="a" rightKey="d" isMotorControlEnable={isMotorControlEnable} />
        <MotorComponent index={2} leftKey="z" rightKey="c" isMotorControlEnable={isMotorControlEnable} />
      </div>
    </div>
  )
}

export default App