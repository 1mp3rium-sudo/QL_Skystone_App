package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.Range
import org.openftc.revextensions2.RevBulkData

class Dead_Wheel(encoder: MA3_Encoder) {
    var encoder = encoder
    var dist = 0.0
    var POC = 0.0

    fun update(data : RevBulkData, dt : Long){
        encoder.update(data, dt)
        dist = encoder.e * 2 * 1.88976
        POC = ((encoder.pos % (Math.PI / 2)) - (Math.PI / 4)) / Math.abs((encoder.pos % (Math.PI / 2)) - (Math.PI / 4))
    }
}