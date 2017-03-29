/*
 * ������: ������������� ������
 * ��������: �������� ����
 * �����: ����� ������ (a.sysoev@synergy.msk.ru)
 *
 * � ���� ����� ���������� ��� ������, ������� ��������� ��������� �����������
 */
#ifndef CLI_TEXT_H
#define CLI_TEXT_H

FlashString CLI_TEXT_UNKNOWN_COMMAND = "\r\n������: ������� %s �� �������!";
FlashString CLI_TEXT_BAD_PARAMETERS = "\r\n������: �� ����� ������ ��������� �������!";
FlashString CLI_TEXT_INVALID_PARAM_VALUE = "\r\n������: �� ����� �� ���������� �������� ��������� %s\r\n������� ����� �� %u �� %u";
FlashString CLI_TEXT_PARAM_OK = "\r\n����� �������� ��������� %s �����������";
FlashString CLI_TEXT_APPLY_EXECUTED = "\r\n����� ��������� ���������. ���� ���������� �������� ���������,\r\n�� ��������� ������� save, ����� ��������� ����� ���������";
FlashString CLI_TEXT_SAVE_EXECUTED = "\r\n������� ��������� ��������� � ����������������� ������";

#ifdef DEBUG

FlashString CLI_TEXT_DEBUG_INFO = "\r\n������� ����� ���������� ���������� ��� ������������:\n\r";
FlashString CLI_TEXT_DEBUG_CLI_TASK_SS = "\r\n������ ����� ������ cliTask = ";
FlashString CLI_TEXT_DEBUG_MODBUS_TASK_SS = "\r\n������ ����� ������ modbusTask = ";
FlashString CLI_TEXT_DEBUG_SENS_TASK_SS = "\r\n������ ����� ������ sensTask = ";

#endif

FlashString CLI_TEXT_HELP_VERSION  = " - ������� ������� ������ ��������������\r\n";
FlashString CLI_TEXT_HELP_GET = " - ������� ��� ��������� ����������, ��������� �������� � ������\r\n";
FlashString CLI_TEXT_HELP_SET = " - ������������� ����� ��������� ����������\r\n";
FlashString CLI_TEXT_HELP_APPLY = " - ��������� ����� ��������� ���������� �\r\n��������� ���������� ���������� � ����� �����������\r\n";
FlashString CLI_TEXT_HELP_APPLY_DEFAULT = " default - ��������� ��������� ���������� �� ��������� �\r\n��������� ���������� ���������� � ����� �����������\r\n";
FlashString CLI_TEXT_HELP_SAVE = " - ��������� ������� ��������� ���������� � ����������������� ������\r\n";

FlashString CLI_TEXT_HELP_GET_SETTINGS = " settings - ������� ��� ��������� ����������\r\n";
FlashString CLI_TEXT_HELP_GET_DRYCONTACTS = " drycontacts - ������� ��������� ���������� ��������\r\n";
FlashString CLI_TEXT_HELP_GET_GAS = " gas - ������� ��������� ������� ��������������\r\n";
FlashString CLI_TEXT_HELP_GET_WATER = " water - ������� ��������� ������� ����������\r\n";
FlashString CLI_TEXT_HELP_GET_FOG = " fog - ������� ��������� ������� ����������\r\n";
FlashString CLI_TEXT_HELP_GET_ERROR = " error - ������� �������� ������ � ������ ����������\r\n";
FlashString CLI_TEXT_HELP_GET_RELAYS = " relays - ������� ��������� ���� �������� ����\r\n";
FlashString CLI_TEXT_HELP_GET_SYSTEM = " system - ������� ��������� ��������� ����������\r\n";
FlashString CLI_TEXT_HELP_GET_TEMPERATURE = " temperature - ������� ���������� �������� ����������� (����. �������)\r\n";

FlashString CLI_TEXT_HELP_SET_MODBUS = " modbus -addr (����� ���������� �� 1 �� 255)\r\n\t-baudrate (�������� ������ �� 2400 �� 115200)\r\n\t-parity (��� �������� 0 - ���, 1 - �����, 2 - ���)\r\n\t-stopbits (���������� �������� ����� �� 0 �� 2)\r\n";
FlashString CLI_TEXT_HELP_SET_GAS = " gas -level (����� ��������� �� 0 �� 4095)\r\n\t-hyst (���������� �� 0 �� 100%)\r\n\t-period (������ ������ �� 0 �� 65535 ������)\r\n\t-warmup (����� �������� �� 0 �� 65535 ������)\r\n\ton ��� off (��������/���������)\r\n";
FlashString CLI_TEXT_HELP_SET_FOG = " fog -level (����� ��������� �� 0 �� 4095)\r\n\t-hyst (���������� �� 0 �� 100%)\r\n\ton ��� off (��������/���������)\r\n";
FlashString CLI_TEXT_HELP_SET_WATER = " water -level (����� ��������� �� 0 �� 4095)\r\n\t-hyst (���������� �� 0 �� 100%)\r\n";
FlashString CLI_TEXT_HELP_SET_DRYCONTACTS = " drycontacts -level (����� ��������� �� 0 �� 4095)\r\n";
FlashString CLI_TEXT_HELP_SET_RELAY = " relay1 ��� relay2 on ��� off (��������� �������� ����)\r\n";

FlashString CLI_TEXT_SETTINGS_CUR = "\r\n������� ��������� ����������:\r\n";
FlashString CLI_TEXT_SETTINGS_NEW = "\r\n\r\n����� ��������� ����������:\r\n";
FlashString CLI_TEXT_SETTINGS_WARN_APPLY = "\r\n\r\n��������: ����� ����� ��������� �������� � ����,\r\n���������� ��������� ������� apply";
FlashString CLI_TEXT_MODBUS_ADDR = "\r\nModbus: ����� ���������� = ";
FlashString CLI_TEXT_MODBUS_BAUDRATE = "\r\nModbus: �������� ������ (���) = ";
FlashString CLI_TEXT_MODBUS_PARITY = "\r\nModbus: ������� ���� �������� = ";
FlashString CLI_TEXT_MODBUS_STOPBITS = "\r\nModbus: ���������� �������� ����� = ";
FlashString CLI_TEXT_GAS_LEVEL = "\r\n���: ����� ��������� = ";
FlashString CLI_TEXT_GAS_HYST = "\r\n���: ������� ����������� = ";
FlashString CLI_TEXT_GAS_PERIOD = "\r\n���: ������ ������ (������) = ";
FlashString CLI_TEXT_GAS_WARMUP = "\r\n���: ����� �������� (������) = ";
FlashString CLI_TEXT_WATER_LEVEL = "\r\n����: ����� ��������� = ";
FlashString CLI_TEXT_WATER_HYST = "\r\n����: ������� ����������� = ";
FlashString CLI_TEXT_FOG_LEVEL = "\r\n���: ����� ��������� = ";
FlashString CLI_TEXT_FOG_HYST = "\r\n���: ������� ����������� = ";
FlashString CLI_TEXT_DRYCONTACTS_LEVEL = "\r\n������: ����� ��������� = ";

FlashString CLI_TEXT_NONE = "���";
FlashString CLI_TEXT_ODD = "��������";
FlashString CLI_TEXT_EVEN = "������";
FlashString CLI_TEXT_UNKNOWN = "�� ����������";
FlashString CLI_TEXT_CONNECTED = "���������";
FlashString CLI_TEXT_DISCONNECTED = "�� ���������";
FlashString CLI_TEXT_OPEN = "������";
FlashString CLI_TEXT_CLOSED = "������";
FlashString CLI_TEXT_CRLF = "\r\n";
FlashString CLI_TEXT_CUR_LEVEL = "������� ������� = ";
FlashString CLI_TEXT_WATER_STATE0 = "��� �������� ����";
FlashString CLI_TEXT_WATER_STATE1 = "���������� �������� ����";
FlashString CLI_TEXT_FOG_STATE0 = "��� ����������";
FlashString CLI_TEXT_FOG_STATE1 = "���������� ���������� ���������";
FlashString CLI_TEXT_GAS_STATE0 = "��� ������ ����";
FlashString CLI_TEXT_GAS_STATE1 = "���������� ������ ����";

FlashString CLI_TEXT_DRYCONTACTS = "���������� ������";
FlashString CLI_TEXT_WATER = "������ ��������";
FlashString CLI_TEXT_GAS = "������ ��������������";
FlashString CLI_TEXT_FOG = "������ ����������";

FlashString CLI_TEXT_RELAY_CHANGED = "����� ��������� ���� �����������";
FlashString CLI_TEXT_RELAY = "����";
FlashString CLI_TEXT_ON = "��������";
FlashString CLI_TEXT_OFF = "���������";
FlashString CLI_TEXT_SHORT_12V = "�� � ���� 12�: ";
FlashString CLI_TEXT_COVER = "������ �������: ";
FlashString CLI_TEXT_USER_BTN = "������ SW001: ";
FlashString CLI_TEXT_YES = "��";
FlashString CLI_TEXT_NO = "���";
FlashString CLI_TEXT_TEMPERATURE = "�����������: ";
FlashString CLI_TEXT_VDD = "Vdd: ";
FlashString CLI_TEXT_MCU_TEMPERATURE = "����������� MCU: ";
FlashString CLI_TEXT_12VIN = "�������� ��� �� ����� 12V: ";
FlashString CLI_TEXT_12VOUT = "�������� ��� ����� ����� 12V: ";
FlashString CLI_TEXT_PWR12V = "��12V: ";
FlashString CLI_TEXT_FOG_ENABLE = "������� ����: ";
FlashString CLI_TEXT_GAS_ENABLE = "������ ����: ";

#endif
