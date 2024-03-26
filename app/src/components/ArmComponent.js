import { useEffect, useRef, useState } from "react";
import useMotorServices from "../hooks/useMotorServices";
import { useRosHook } from "../contexts/RosConnectionContext";
import useMotor from "../hooks/useMotor";

export default function ArmComponent() {
    const { rosStatus } = useRosHook();
    const { getMotorJointsLenght } = useMotorServices();

    const [jointsLength, setJointsLength] = useState([0, 0, 0]);

    const {position: position1} = useMotor(0)
    const {position: position2} = useMotor(1)
    const {position: position3} = useMotor(2)

    const canvasRef = useRef(null);

    useEffect(() => {
        if (rosStatus.status === "connected") {
            (async () => {
                setJointsLength(await getMotorJointsLenght());
            })()
        }

    }, [rosStatus])


    useEffect(() => {
        if (jointsLength[0] === 0 || jointsLength[1] === 0 || jointsLength[2] === 0) {
            return
        }

        const ctx = canvasRef.current.getContext("2d");

        const { width, height } = canvasRef.current.getBoundingClientRect();
    
        ctx.clearRect(0, 0, width, height)
        ctx.strokeStyle = "white"
        ctx.fillStyle = "white"


        let direction = directionVector(position1)
        let vector = scaleVector(direction, jointsLength[0])
        let endPosition1 = moveVector([width/2, height/2], vector)

        drawCirlce(ctx, width/2, height/2)
        drawLine(ctx, width/2, height/2, endPosition1[0], endPosition1[1])

        direction = directionVector(position2)
        vector = scaleVector(direction, jointsLength[1])
        let endPosition2 = moveVector(endPosition1, vector)
        
        drawCirlce(ctx, endPosition1[0], endPosition1[1])
        drawLine(ctx, endPosition1[0], endPosition1[1], endPosition2[0], endPosition2[1])


        direction = directionVector(position3)
        vector = scaleVector(direction, jointsLength[2])
        let endPosition3 = moveVector(endPosition2, vector)
        
        drawCirlce(ctx, endPosition2[0], endPosition2[1])
        drawLine(ctx, endPosition2[0], endPosition2[1], endPosition3[0], endPosition3[1])


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
        const step = 2 * Math.PI / 4095
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
        <div>
            <canvas width="1200px" height="1200px" ref={canvasRef}></canvas>
        </div>
    )
}