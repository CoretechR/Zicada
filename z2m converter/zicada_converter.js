import * as fz from "zigbee-herdsman-converters/converters/fromZigbee";
import * as exposes from "zigbee-herdsman-converters/lib/exposes";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";

const e = exposes.presets;

// Custom fromZigbee converter for contact events
const fz_command_onoff_contact = {
    cluster: "genOnOff",
    type: ["commandOn", "commandOff"],
    convert: (model, msg, publish, options, meta) => {
        if (msg.type === "commandOn") {
            return {contact: false};
        }
        if (msg.type === "commandOff") {
            return {contact: true};
        }
    },
};

export default {
    fingerprint: [{modelID: "Zicada", manufacturerName: "kernm.de"}],
    model: "Zicada",
    vendor: "kernm.de",
    description: "Multisensor with temperature, humidity, and contact sensors",
    fromZigbee: [fz.temperature, fz.humidity, fz.battery, fz_command_onoff_contact],
    toZigbee: [],
    exposes: [e.temperature(), e.humidity(), e.battery(), e.contact()],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
		await reporting.bind(
		endpoint,
		coordinatorEndpoint, [
			"genPowerCfg",
            "msTemperatureMeasurement",
			"msRelativeHumidity"
		]);
        await reporting.batteryPercentageRemaining(endpoint);
		await reporting.temperature(endpoint);
		await reporting.humidity(endpoint);		
    },
};
