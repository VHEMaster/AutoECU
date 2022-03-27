/*
 * failures.c
 *
 *  Created on: 24 мар. 2022 г.
 *      Author: VHEMaster
 */


const char * const gCheckDataErrorString[] = {
    "Flash: Load failure",
    "Flash: Save failure",
    "Flash: Init failure",
    "Bkpsram: Save failure",
    "Bkpsram: Load failure",
    "Sensor: MAP failure",
    "Sensor: Knock failure",
    "Sensor: CSPS failure",
    "Sensor: TSPS failure",
    "Sensor: AirTemp failure",
    "Sensor: EngineTemp failure",
    "Sensor: TPS failure",
    "Sensor: RefVoltage failure",
    "Sensor: PwrVoltage failure",
    "Sensor: Lambda failure",

    "Output: Driver failure",

    "Injector 4: OpenCircuit",
    "Injector 4: ShortToBatOrOverheat",
    "Injector 4: ShortToGND",
    "Injector 3: OpenCircuit",
    "Injector 3: ShortToBatOrOverheat",
    "Injector 3: ShortToGND",
    "Injector 2: OpenCircuit",
    "Injector 2: ShortToBatOrOverheat",
    "Injector 2: ShortToGND",
    "Injector 1: OpenCircuit",
    "Injector 1: ShortToBatOrOverheat",
    "Injector 1: ShortToGND",
    "Injector: Communication failure",

    "CheckEngine: OpenCircuit",
    "CheckEngine: ShortToBatOrOverheat",
    "CheckEngine: ShortToGND",
    "SpeedMeter: OpenCircuit",
    "SpeedMeter: ShortToBatOrOverheat",
    "SpeedMeter: ShortToGND",
    "Tachometer: OpenCircuit",
    "Tachometer: ShortToBatOrOverheat",
    "Tachometer: ShortToGND",
    "FuelPump: OpenCircuit",
    "FuelPump: ShortToBatOrOverheat",
    "FuelPump: ShortToGND",
    "Outputs1: Communication failure",

    "OutRsvd2: OpenCircuit",
    "OutRsvd2: ShortToBatOrOverheat",
    "OutRsvd2: ShortToGND",
    "OutRsvd1: OpenCircuit",
    "OutRsvd1: ShortToBatOrOverheat",
    "OutRsvd1: ShortToGND",
    "StarterRelay: OpenCircuit",
    "StarterRelay: ShortToBatOrOverheat",
    "StarterRelay: ShortToGND",
    "FanRelay: OpenCircuit",
    "FanRelay: ShortToBatOrOverheat",
    "FanRelay: ShortToGND",
    "Outputs2: Communication failure",

    "IdleValve: Failure",
    "IdleValve: Driver failure",
    "Injection: Fuel underflow",

    "Lambda: Communication failure",
    "Lambda: VM ShortToBat",
    "Lambda: VM LowBattery",
    "Lambda: VM ShortToGnd",
    "Lambda: UN ShortToBat",
    "Lambda: UN LowBattery",
    "Lambda: UN ShortToGnd",
    "Lambda: IAIP ShortToBat",
    "Lambda: IAIP LowBattery",
    "Lambda: IAIP ShortToGnd",
    "Lambda: DIAHGD ShortToBat",
    "Lambda: DIAHGD OpenCircuit",
    "Lambda: DIAHGD ShortToGnd",

    "Knock: Detonation Found",
    "Knock: Low Noise Level",

};
