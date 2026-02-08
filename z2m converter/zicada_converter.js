const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');

const e = exposes.presets;

// Local definition to avoid import errors
const fz_local_illuminance = {
    cluster: 'msIlluminanceMeasurement',
    type: ['attributeReport', 'readResponse'],
    options: [],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.hasOwnProperty('measuredValue')) {
            const measuredValue = msg.data['measuredValue'];
            const lux = Math.pow(10, measuredValue / 10000) - 1;
            return { illuminance: Math.round(lux) };
        }
    },
};

const fz_command_onoff_contact = {
    cluster: 'genOnOff',
    type: ['commandOn', 'commandOff'],
    options: [],
    convert: (model, msg, publish, options, meta) => {
        if (msg.type === 'commandOn') return { contact: false };
        if (msg.type === 'commandOff') return { contact: true };
    },
};

module.exports = {
    fingerprint: [{ modelID: 'Zicada', manufacturerName: 'kernm.de' }],
    model: 'Zicada',
    vendor: 'kernm.de',
    description: 'Multisensor with temperature, humidity, illuminance and contact sensors',
    
    fromZigbee: [
        fz.temperature, 
        fz.humidity, 
        fz_local_illuminance,
        fz.battery, 
        fz_command_onoff_contact
    ],
    
    toZigbee: [],
    
    exposes: [
        e.temperature(), 
        e.humidity(), 
        e.illuminance(), 
        e.battery(), 
        e.contact()
    ],
    
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        
        await reporting.bind(endpoint, coordinatorEndpoint, [
            'genPowerCfg',
            'msTemperatureMeasurement',
            'msRelativeHumidity',
            'msIlluminanceMeasurement'
        ]);

        await reporting.batteryPercentageRemaining(endpoint, { min: 300, max: 21600, change: 1 }); // Battery every 6h
        await reporting.temperature(endpoint, { min: 300, max: 900, change: 10 });
        await reporting.humidity(endpoint, { min: 300, max: 900, change: 100 });
        await reporting.illuminance(endpoint, { min: 300, max: 900, change: 5 });
    },
};