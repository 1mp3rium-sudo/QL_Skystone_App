package org.firstinspires.ftc.teamcode

import org.openftc.revextensions2.RevBulkData

class MA3_Encoder(port : Int, offset : Double) {
    var e = 0.0
    var pos = 0.0
    val port = port
    val offset = offset
    var prev_velo = 0.0

    val MAX_ACCEL = 60

    fun update(data : RevBulkData, dt : Long) {
        pos = (data.getAnalogInputValue(port).toDouble() * (2 * Math.PI / 3.3) - offset + (2 * Math.PI)) % (2 * Math.PI)
        if ((Math.abs(pos - e) - prev_velo) / dt > MAX_ACCEL) {
            val offset = pos - (e % (2 * Math.PI)) + (2 * Math.PI)
            e += offset
            prev_velo = offset / dt
        } else {
            val offset = pos - (e % (2 * Math.PI))
            e += offset
            prev_velo = offset / dt
        }
        pos = data.getAnalogInputValue(port).toDouble() * (2 * Math.PI / 3.3)
    }
}