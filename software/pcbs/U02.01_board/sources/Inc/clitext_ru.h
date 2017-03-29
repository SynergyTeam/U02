/*
 * Проект: Универсальный Датчик
 * Компания: Синерджи Тиам
 * Автор: Антон Сысоев (a.sysoev@synergy.msk.ru)
 *
 * В этом файле содержатся все строки, которые выводятся командным интерфейсом
 */
#ifndef CLI_TEXT_H
#define CLI_TEXT_H

FlashString CLI_TEXT_UNKNOWN_COMMAND = "\r\nОшибка: команда %s не найдена!";
FlashString CLI_TEXT_BAD_PARAMETERS = "\r\nОшибка: не верно заданы параметры команды!";
FlashString CLI_TEXT_INVALID_PARAM_VALUE = "\r\nОшибка: вы ввели не допустимое значение параметра %s\r\nВведите число от %u до %u";
FlashString CLI_TEXT_PARAM_OK = "\r\nНовое значение параметра %s установлено";
FlashString CLI_TEXT_APPLY_EXECUTED = "\r\nНовые настройки применены. Если устройство работает нормально,\r\nто выполните команду save, чтобы сохранить новые настройки";
FlashString CLI_TEXT_SAVE_EXECUTED = "\r\nТекущие настройки сохранены в энергонезависимую память";

#ifdef DEBUG

FlashString CLI_TEXT_DEBUG_INFO = "\r\nНачинаю вывод отладочной информации для программиста:\n\r";
FlashString CLI_TEXT_DEBUG_CLI_TASK_SS = "\r\nРазмер стека задачи cliTask = ";
FlashString CLI_TEXT_DEBUG_MODBUS_TASK_SS = "\r\nРазмер стека задачи modbusTask = ";
FlashString CLI_TEXT_DEBUG_SENS_TASK_SS = "\r\nРазмер стека задачи sensTask = ";

#endif

FlashString CLI_TEXT_HELP_VERSION  = " - Выводит текущую версию микропрограммы\r\n";
FlashString CLI_TEXT_HELP_GET = " - Выводит все настройки устройства, состояние датчиков и прочее\r\n";
FlashString CLI_TEXT_HELP_SET = " - Устанавливает новые настройки устройства\r\n";
FlashString CLI_TEXT_HELP_APPLY = " - Применяет новые настройки устройства и\r\nвыполняет перезапуск устройства с этими настройками\r\n";
FlashString CLI_TEXT_HELP_APPLY_DEFAULT = " default - Применяет настройки устройства по умолчанию и\r\nвыполняет перезапуск устройства с этими настройками\r\n";
FlashString CLI_TEXT_HELP_SAVE = " - Сохраняет текущие настройки устройства в энергонезависимую память\r\n";

FlashString CLI_TEXT_HELP_GET_SETTINGS = " settings - Выводит все настройки устройства\r\n";
FlashString CLI_TEXT_HELP_GET_DRYCONTACTS = " drycontacts - Выводит состояние герконовых датчиков\r\n";
FlashString CLI_TEXT_HELP_GET_GAS = " gas - Выводит состояние датчика загазованности\r\n";
FlashString CLI_TEXT_HELP_GET_WATER = " water - Выводит состояние датчика затопления\r\n";
FlashString CLI_TEXT_HELP_GET_FOG = " fog - Выводит состояние датчика задымления\r\n";
FlashString CLI_TEXT_HELP_GET_ERROR = " error - Выводит описание ошибки в работе устройства\r\n";
FlashString CLI_TEXT_HELP_GET_RELAYS = " relays - Выводит состояние всех выходных реле\r\n";
FlashString CLI_TEXT_HELP_GET_SYSTEM = " system - Выводит состояние системных переменных\r\n";
FlashString CLI_TEXT_HELP_GET_TEMPERATURE = " temperature - Выводит измеренное значение температуры (град. Цельсия)\r\n";

FlashString CLI_TEXT_HELP_SET_MODBUS = " modbus -addr (Адрес устройства от 1 до 255)\r\n\t-baudrate (Скорость обмена от 2400 до 115200)\r\n\t-parity (Бит четности 0 - нет, 1 - нечет, 2 - чет)\r\n\t-stopbits (Количество стоповых битов от 0 до 2)\r\n";
FlashString CLI_TEXT_HELP_SET_GAS = " gas -level (Порог сравнения от 0 до 4095)\r\n\t-hyst (Гистерезис от 0 до 100%)\r\n\t-period (Период опроса от 0 до 65535 секунд)\r\n\t-warmup (Время прогрева от 0 до 65535 секунд)\r\n\ton или off (включить/выключить)\r\n";
FlashString CLI_TEXT_HELP_SET_FOG = " fog -level (Порог сравнения от 0 до 4095)\r\n\t-hyst (Гистерезис от 0 до 100%)\r\n\ton или off (включить/выключить)\r\n";
FlashString CLI_TEXT_HELP_SET_WATER = " water -level (Порог сравнения от 0 до 4095)\r\n\t-hyst (Гистерезис от 0 до 100%)\r\n";
FlashString CLI_TEXT_HELP_SET_DRYCONTACTS = " drycontacts -level (Порог сравнения от 0 до 4095)\r\n";
FlashString CLI_TEXT_HELP_SET_RELAY = " relay1 или relay2 on или off (Состояние выходных реле)\r\n";

FlashString CLI_TEXT_SETTINGS_CUR = "\r\nТекущие настройки устройства:\r\n";
FlashString CLI_TEXT_SETTINGS_NEW = "\r\n\r\nНовые настройки устройства:\r\n";
FlashString CLI_TEXT_SETTINGS_WARN_APPLY = "\r\n\r\nВнимание: чтобы новые настройки вступили в силу,\r\nнеобходимо выполнить команду apply";
FlashString CLI_TEXT_MODBUS_ADDR = "\r\nModbus: Адрес устройства = ";
FlashString CLI_TEXT_MODBUS_BAUDRATE = "\r\nModbus: Скорость обмена (бод) = ";
FlashString CLI_TEXT_MODBUS_PARITY = "\r\nModbus: Наличие бита четности = ";
FlashString CLI_TEXT_MODBUS_STOPBITS = "\r\nModbus: Количество стоповых битов = ";
FlashString CLI_TEXT_GAS_LEVEL = "\r\nГаз: Порог сравнения = ";
FlashString CLI_TEXT_GAS_HYST = "\r\nГаз: Уровень гистерезиса = ";
FlashString CLI_TEXT_GAS_PERIOD = "\r\nГаз: Период опроса (секунд) = ";
FlashString CLI_TEXT_GAS_WARMUP = "\r\nГаз: Время прогрева (секунд) = ";
FlashString CLI_TEXT_WATER_LEVEL = "\r\nВода: Порог сравнения = ";
FlashString CLI_TEXT_WATER_HYST = "\r\nВода: Уровень гистерезиса = ";
FlashString CLI_TEXT_FOG_LEVEL = "\r\nДым: Порог сравнения = ";
FlashString CLI_TEXT_FOG_HYST = "\r\nДым: Уровень гистерезиса = ";
FlashString CLI_TEXT_DRYCONTACTS_LEVEL = "\r\nГеркон: Порог сравнения = ";

FlashString CLI_TEXT_NONE = "Нет";
FlashString CLI_TEXT_ODD = "Нечетный";
FlashString CLI_TEXT_EVEN = "Четный";
FlashString CLI_TEXT_UNKNOWN = "Не определено";
FlashString CLI_TEXT_CONNECTED = "Подключен";
FlashString CLI_TEXT_DISCONNECTED = "Не подключен";
FlashString CLI_TEXT_OPEN = "Открыт";
FlashString CLI_TEXT_CLOSED = "Закрыт";
FlashString CLI_TEXT_CRLF = "\r\n";
FlashString CLI_TEXT_CUR_LEVEL = "Текущий уровень = ";
FlashString CLI_TEXT_WATER_STATE0 = "Нет протечки воды";
FlashString CLI_TEXT_WATER_STATE1 = "ОБНАРУЖЕНА ПРОТЕЧКА ВОДЫ";
FlashString CLI_TEXT_FOG_STATE0 = "Нет задымления";
FlashString CLI_TEXT_FOG_STATE1 = "ОБНАРУЖЕНО ЗАДЫМЛЕНИЕ ПОМЕЩЕНИЯ";
FlashString CLI_TEXT_GAS_STATE0 = "Нет утечки газа";
FlashString CLI_TEXT_GAS_STATE1 = "ОБНАРУЖЕНА УТЕЧКА ГАЗА";

FlashString CLI_TEXT_DRYCONTACTS = "Герконовый датчик";
FlashString CLI_TEXT_WATER = "Датчик протечки";
FlashString CLI_TEXT_GAS = "Датчик загазованности";
FlashString CLI_TEXT_FOG = "Датчик задымления";

FlashString CLI_TEXT_RELAY_CHANGED = "Новое состояние реле установлено";
FlashString CLI_TEXT_RELAY = "Реле";
FlashString CLI_TEXT_ON = "Включено";
FlashString CLI_TEXT_OFF = "Выключено";
FlashString CLI_TEXT_SHORT_12V = "КЗ в цепи 12В: ";
FlashString CLI_TEXT_COVER = "Корпус прибора: ";
FlashString CLI_TEXT_USER_BTN = "Кнопка SW001: ";
FlashString CLI_TEXT_YES = "Да";
FlashString CLI_TEXT_NO = "Нет";
FlashString CLI_TEXT_TEMPERATURE = "Температура: ";
FlashString CLI_TEXT_VDD = "Vdd: ";
FlashString CLI_TEXT_MCU_TEMPERATURE = "Температура MCU: ";
FlashString CLI_TEXT_12VIN = "Значение АЦП до шунта 12V: ";
FlashString CLI_TEXT_12VOUT = "Значение АЦП после шунта 12V: ";
FlashString CLI_TEXT_PWR12V = "БП12V: ";
FlashString CLI_TEXT_FOG_ENABLE = "Датчики дыма: ";
FlashString CLI_TEXT_GAS_ENABLE = "Датчик газа: ";

#endif
