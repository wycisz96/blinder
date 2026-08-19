const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;

const ROLETA_ENDPOINT = 10;

// Timeout musi być < 7.68s (bufor indirect transmission koordynatora),
// żeby każda kolejna próba faktycznie wypadła w nowym oknie bufora.
const COMMAND_TIMEOUT_MS = 7000;

// 6 prób * 7s = 42s > 30s cyklu snu urządzenia -> gwarantuje trafienie
// w okno nasłuchu w ciągu jednego pełnego cyklu, z zapasem.
const COMMAND_RETRIES = 6;
const RETRY_DELAY_MS = 0; // sam timeout daje właściwy odstęp między próbami

const READ_TIMEOUT_MS = 9000;
const READ_RETRIES = 1;
const READ_RETRY_DELAY_MS = 0;

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * Wysyła komendę Zigbee z ponawianiem na poziomie protokołu.
 * entity.command() rezolwuje się dopiero po realnym potwierdzeniu dostarczenia
 * (APS/data confirm) dla sleepy end device — więc sukces tutaj = komenda
 * faktycznie dotarła do urządzenia, nie tylko "wysłana z Z2M".
 * Rzuca błąd po wyczerpaniu prób, żeby Z2M/HA wiedziały, że się nie udało
 * (żadnego cichego połykania błędów, żadnego optimistic state wyżej w stosie).
 */
async function sendWithRetry(entity, cluster, command, payload, logger) {
    let lastError;
    for (let attempt = 1; attempt <= COMMAND_RETRIES; attempt++) {
        try {
            await entity.command(cluster, command, payload, { timeout: COMMAND_TIMEOUT_MS });
            if (logger && attempt > 1) {
                logger.debug(`Roleta: komenda '${command}' dostarczona przy próbie ${attempt}/${COMMAND_RETRIES}`);
            }
            return;
        } catch (error) {
            lastError = error;
            if (logger) {
                logger.debug(`Roleta: próba ${attempt}/${COMMAND_RETRIES} komendy '${command}' nieudana: ${error.message}`);
            }
            if (attempt < COMMAND_RETRIES && RETRY_DELAY_MS > 0) await sleep(RETRY_DELAY_MS);
        }
    }
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
            return { position: 100 - msg.data.currentPositionLiftPercentage };
        }
    },
};

const tzRoleta = {
    key: ['position', 'state'],
    convertSet: async (entity, key, value, meta) => {
        if (key === 'position')
	 {
	    const zigbeeValue = 100 - value;
            await sendWithRetry(entity, 'closuresWindowCovering', 'goToLiftPercentage', { percentageliftvalue: zigbeeValue }, meta.logger);
            // Brak optimistic echo — stan publikuje wyłącznie fzRoleta na podstawie
            // realnego attributeReport z urządzenia.
            return;
        }
        if (key === 'state') {
            const upper = String(value).toUpperCase();
            if (upper === 'OPEN') {
                // OPEN = jedź do 0% bez fizycznego homingu (goToLiftPercentage zamiast upOpen)
                // upOpen jest zarezerwowane dla kalibracji
                await sendWithRetry(entity, 'closuresWindowCovering', 'goToLiftPercentage', { percentageliftvalue: 0 }, meta.logger);
            } else {
                const lookup = { CLOSE: 'downClose', STOP: 'stop' };
                const command = lookup[upper];
                if (command) {
                    await sendWithRetry(entity, 'closuresWindowCovering', command, {}, meta.logger);
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
        // Wysyła upOpen (cmd 0x00) jako trigger kalibracji.
        // W firmware upOpen = MOTOR_CMD_CALIBRATE: fizyczny ruch do krańcówki + wyzerowanie pozycji.
        // Zwykłe przejście do 0% realizuje goToLiftPercentage(0) (patrz tzRoleta state=OPEN).
        await sendWithRetry(entity, 'closuresWindowCovering', 'upOpen', {}, meta.logger);
        // ACK przychodzi jako attributeReport pozycji po zakończeniu homingu
        // w firmware (ten sam mechanizm raportowania co po zwykłym ruchu).
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
            min: 1,        // min 1 s między raportami
            max: 3600,     // max 1h (heartbeat)
            change: 1,     // raportuj każdą zmianę o ≥1%
        });
    },
};

module.exports = definition;