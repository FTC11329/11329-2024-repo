package org.firstinspires.ftc.teamcode.utility;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Map;

public class CustomHardwareMap extends HardwareMap {

    public CustomHardwareMap(Context appContext, OpModeManagerNotifier notifier) {
        super(appContext, notifier);
    }

    public Map<String, List<HardwareDevice>>  getMagicHardwareMapStuff() {
        return this.allDevicesMap;
    }
}
