const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;

const ROLETA_ENDPOINT = 10;

const COMMAND_TIMEOUT_MS = 7000; // < 7.68s bufora koordynatora
const COMMAND_RETRIES = 6;       // 6*7s ≈ 42s > 30s cyklu snu
const RETRY_DELAY_MS = 0;

const READ_TIMEOUT_MS = 9000;
const READ_RETRIES = 1;
const READ_RETRY_DELAY_MS = 0;

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * Licznik generacji komend per urządzenie (IEEE address).
 * Każde nowe zlecenie z HA (position, state, calibrate — wspólna przestrzeń,
 * bo to ten sam silnik) dostaje kolejny numer i unieważnia retry-pętle
 * poprzednich, wciąż trwających zleceń. Zapobiega sytuacji, w której stara,
 * opóźniona retransmisja w końcu dociera do urządzenia PO nowszej komendzie
 * i nadpisuje jej efekt.
 */
const commandGeneration = new Map(); // ieeeAddr -> ostatni wydany numer generacji

function nextGeneration(ieeeAddr) {
    const gen = (commandGeneration.get(ieeeAddr) || 0) + 1;
    commandGeneration.set(ieeeAddr, gen);
    return gen;
}

function isSuperseded(ieeeAddr, generation) {
    return commandGeneration.get(ieeeAddr) !== generation;
}

/**
 * Wysyła komendę Zigbee z ponawianiem na poziomie protokołu, przerywając
 * retry jeśli w międzyczasie nadeszła nowsza komenda (inna generacja).
 * Porzucenie z powodu bycia "superseded" NIE jest traktowane jako błąd —
 * to oczekiwane zachowanie przy "ostatnia komenda wygrywa".
 */
async function sendWithRetry(entity, cluster, command, payload, logger, generation) {
    const ieeeAddr = entity.deviceIeeeAddress;
    let lastError;
    for (let attempt = 1; attempt <= COMMAND_RETRIES; attempt++) {
        if (isSuperseded(ieeeAddr, generation)) {
            console.log(`Roleta: komenda '${command}' (gen ${generation}) porzucona — nadpisana nowszą komendą`);
	    if (logger) {
                logger.info(`Roleta: komenda '${command}' (gen ${generation}) porzucona — nadpisana nowszą komendą`);
            }
            return;
        }
        try {
            await entity.command(cluster, command, payload, { timeout: COMMAND_TIMEOUT_MS });
            if (logger && attempt > 1) {
                logger.info(`Roleta: komenda '${command}' dostarczona przy próbie ${attempt}/${COMMAND_RETRIES}`);
            }
            return;
        } catch (error) {
            lastError = error;
            if (logger) {
                logger.info(`Roleta: próba ${attempt}/${COMMAND_RETRIES} komendy '${command}' nieudana: ${error.message}`);
            }
            if (attempt < COMMAND_RETRIES && RETRY_DELAY_MS > 0) await sleep(RETRY_DELAY_MS);
        }
    }
    if (isSuperseded(ieeeAddr, generation)) return; // ostatnia próba i tak już nieaktualna
    const msg = `Roleta: komenda '${command}' nie dotarła po ${COMMAND_RETRIES} próbach: ${lastError.message}`;
    if (logger) logger.error(msg);
    throw new Error(msg);
}

async function readWithRetry(entity, cluster, attributes, logger) {
    let lastError;
    for (let attempt = 1; attempt <= READ_RETRIES; attempt++) {
        try {
            return await entity.read(cluster, attributes, { timeout: READ_TIMEOUT_MS });
        } catch (error) {
            lastError = error;
            if (attempt < READ_RETRIES) await sleep(READ_RETRY_DELAY_MS);
        }
    }
    const msg = `Roleta: odczyt '${attributes}' nieudany: ${lastError.message}`;
    if (logger) logger.warning(msg);
    throw new Error(msg);
}

const fzRoleta = {
    cluster: 'closuresWindowCovering',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        if (msg.data.hasOwnProperty('currentPositionLiftPercentage')) {
            // ZCL: 0=otwarta, 100=zamknięta -> HA: 100=otwarta, 0=zamknięta
            return { position: 100 - msg.data.currentPositionLiftPercentage };
        }
    },
};

const tzRoleta = {
    key: ['position', 'state'],
    convertSet: async (entity, key, value, meta) => {
        const ieeeAddr = entity.deviceIeeeAddress;
        if (key === 'position') {
            const generation = nextGeneration(ieeeAddr);
            const zigbeeValue = 100 - value; // HA -> ZCL
            await sendWithRetry(entity, 'closuresWindowCovering', 'goToLiftPercentage', { percentageliftvalue: zigbeeValue }, meta.logger, generation);
            return;
        }
        if (key === 'state') {
            const generation = nextGeneration(ieeeAddr);
            const upper = String(value).toUpperCase();
            if (upper === 'OPEN') {
                await sendWithRetry(entity, 'closuresWindowCovering', 'goToLiftPercentage', { percentageliftvalue: 0 }, meta.logger, generation);
            } else {
                const lookup = { CLOSE: 'downClose', STOP: 'stop' };
                const command = lookup[upper];
                if (command) {
                    await sendWithRetry(entity, 'closuresWindowCovering', command, {}, meta.logger, generation);
                }
            }
            return;
        }
    },
    convertGet: async (entity, key, meta) => {
        if (key === 'position') {
            await readWithRetry(entity, 'closuresWindowCovering', ['currentPositionLiftPercentage'], meta.logger);
        }
    }
};

const tzCalibrate = {
    key: ['calibrate'],
    convertSet: async (entity, key, value, meta) => {
        const ieeeAddr = entity.deviceIeeeAddress;
        // Wspólna przestrzeń generacji z tzRoleta — kalibracja i ruch do
        // pozycji dzielą ten sam silnik, więc nowa komenda dowolnego typu
        // musi unieważniać poprzednią, niezależnie od typu.
        const generation = nextGeneration(ieeeAddr);
        await sendWithRetry(entity, 'closuresWindowCovering', 'upOpen', {}, meta.logger, generation);
        return;
    },
};

const definition = {
    zigbeeModel: ['ROLETA_ZB_V1', 'esp32c6'],
    model: 'ROLETA_ZB_V1',
    vendor: 'CUSTOM_HW',
    description: 'Wlasny sterownik rolety (ESP32-C6 + A4988)',
    extend: [],
    exposes: [
        e.cover().withPosition(),
        exposes.enum('calibrate', exposes.access.SET, ['START'])
            .withDescription('Uruchom kalibrację: roleta jedzie do krańcówki górnej i zeruje pozycję.'),
    ],
    toZigbee: [tzRoleta, tzCalibrate],
    fromZigbee: [fzRoleta],
    endpoint: (device) => ({ default: ROLETA_ENDPOINT }),
    configure: async (device, coordinatorEndpoint, logger) => {
        const ep = device.getEndpoint(ROLETA_ENDPOINT);
        await reporting.bind(ep, coordinatorEndpoint, ['closuresWindowCovering']);
        await reporting.currentPositionLiftPercentage(ep, {
            min: 1,
            max: 3600,
            change: 1,
        });
    },
};

module.exports = definition;
