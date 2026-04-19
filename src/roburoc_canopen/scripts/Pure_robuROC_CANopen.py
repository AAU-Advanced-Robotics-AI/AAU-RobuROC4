#!/usr/bin/env python3
"""
Created for Lunar surface preparation project
9. Semester, Robotics, 2024 Fall Semester
Group 962

Copyright 2024 Thomas Schou Sørensen, Julian Witold Wagner and César Zacharie G Hostens

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

import canopen, logging, os, asyncio
from time import sleep


from rclpy.node import Node


# Make directory for logs locally
if not os.path.join(os.curdir, "logs"):
    os.mkdir("logs", 0o777)

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    handlers=[logging.FileHandler("CANOpen_Ros_Logs"),
                              logging.StreamHandler()])

class RobuROC_Canopen():
    """
    The RobuROC_Canopen class is constructed to communicate with the DPCANTR-040B080 drives installed in the
    RobuROC4. The methods implemented here are defined such that the low level CANBus communication is simplified to the
    extent possible. When utilized, it will establish and maintain communication with these drivers.
    """

    # _SDO_ABORT_CODES  = {
    #     0x00000503: "Toggle bit not alternated",
    #     0x00000504: "SDO protocol timed out",
    #     0x00010504: "Command specifier not valid",
    #     0x00020504: "Invalid block size (block mode only, see DS301)",
    #     0x00030504: "Invalid sequence number (block mode only, see DS301)",
    #     0x00040504: "CRC error (block mode only, see DS301)",
    #     0x00050504: "Out of memory",
    #     0x00000601: "Unsupported access to an object",
    #     0x00010601: "Attempt to read a write only object",
    #     0x00020601: "Attempt to write a read only object",
    #     0x00000602: "Object does not exist in the object dictionary",
    #     0x00410604: "Object cannot be mapped to the PDO",
    #     0x00420604: "The number and length of the objects to be mapped would exceed PDO length",
    #     0x00430604: "General parameter incompatibility reason",
    #     0x00470604: "General internal incompatibility in the device",
    #     0x00000606: "Access failed due to a hardware error",
    #     0x00100607: "Data type does not match, length of service parameter does not match",
    #     0x00120607: "Data type does not match, length of service parameter too high",
    #     0x00130607: "Data type does not match, length of service parameter too low",
    #     0x00110609: "Sub-index does not exist",
    #     0x00300609: "Value range of parameter exceeded (only for write access)",
    #     0x00310609: "Value of parameter written too high",
    #     0x00320609: "Value of parameter written too low",
    #     0x00360609: "Maximum value is less than minimum value",
    #     0x00000800: "General error",
    #     0x00200800: "Data cannot be transferred or stored to the application",
    #     0x00210800: "Data cannot be transferred or stored to the application because of local control",
    #     0x00220800: "Data cannot be transferred or stored to the application because of present device state",
    #     0x00230800: "Object dictionary dynamic generation fails or no object dictionary is present (object dictionary loads from file and file error occurred)"
    # }
    _DRIVE_CONFIG = os.path.join(os.path.abspath("../../"), "resource", "AMC_Digiflex_1.0.14.eds")  # Adapt as needed

    CAN_NETWORK = canopen.Network()

    def __init__(self):
        self.logger = logging.getLogger("CANBUS")

        # Initialize a network object
        self.CAN_NODES = []
        self._CAN_SUBSCRIPTION = []
        self._CAN_PERIODICTASK = {}
        self._CONNECTED = False

        self._PERIODIC_ID = 0

    def Connect(self, bustype:str = 'pcan', channel:str = 'PCAN_USBBUS1', bitrate:int = 1000000):
        """
        Connection method for attempting to connect to the CANBUS network specified by bustype and channel at the specified
        bitrate.
        :param bustype: type of CANBus
        :param channel: Connection channel (or object), defined bu the interface to the CANBus
        :param bitrate: The bitrate for the connection
        :return: Bool: true or false for connection
        """
        if not self._CONNECTED:
            try:
                self.CAN_NETWORK.connect(bustype=bustype, channel=channel, bitrate=bitrate)
                self._CONNECTED = True
                if len(self.CAN_NODES) == 0:
                    try:
                        # Need at least four nodes to ensure all drives are connected
                        while len(self.CAN_NETWORK.scanner.nodes) < 4:
                            self.CAN_NETWORK.scanner.search()
                            sleep(1)  # Provide the scanner time to find nodes
                        node_list = []
                        for node in self.CAN_NETWORK.scanner.nodes:
                            node = self.CAN_NETWORK.add_node(node, self._DRIVE_CONFIG)
                            self.CAN_NODES.append(node)
                            node_list.append(node.id)
                        self.logger.info(f"Connected to CANBus with nodes: {node_list}")
                    except Exception as e:
                        self.logger.error(f"Unable to initialize nodes, Error: {e}")
                        self._CONNECTED = False
            except Exception as error:
                self.logger.error(f"Unable to connect to CAN Bus, Error: {error}")
                self._CONNECTED = False
            finally:
                return self._CONNECTED
    def Disconnect(self):
        """
        Disconnection method for disconnection the CANBUS network and stopping/ending all
        related subscriptions and periodic tasks
        :return: Bool: True or False for successful disconnection
        """
        if self._CONNECTED:
            try:
                self.CAN_NETWORK.disconnect()
                # It is here assumed that the HeartBeat and Periodic status checks are enabled
                if len(self._CAN_PERIODICTASK) != 0:
                    for task in self._CAN_PERIODICTASK.copy():
                        self._CAN_PERIODICTASK[task].stop()
                        del self._CAN_PERIODICTASK[task]
                if len(self._CAN_SUBSCRIPTION) != 0:
                    for ids in self._CAN_SUBSCRIPTION:
                        self.Unsubscribe(ids)
            except Exception as error:
                self.logger.error(f"Unable to disconnect from CAN Bus, Error: {error}")
            finally:
                self.CAN_NODES.clear()
                self.CAN_NETWORK.scanner.nodes.clear()
                return not self._CONNECTED
        else:
            return True
    def PeriodicTask(self, cobid: int, data: list, period: float = 0.2):
        """
        Adds a CANOpen periodically sent message, e.g. for heartbeat. This task is stored should it later be
        required to be stopped.
        :param cobid: COBID of the intended target
        :param data: Data to be sent periodically
        :param period: The period between messages in seconds
        :return: Bool: True or False for successful periodic task addition
        """
        data = list(data)
        sent_data = None
        if len(data) <= 8:
            # Zero-pad data if necessary
            data += [0] * (8 - len(data))
            data.reverse()
            sent_data = bytearray(data)
        try:
            task = self.CAN_NETWORK.send_periodic(cobid, sent_data, period)
            self._CAN_PERIODICTASK.setdefault(self._PERIODIC_ID, task)
            self._PERIODIC_ID += 1
            return True
        except Exception as error:
            self.logger.error(f"Unable to send periodic task, Error: {error}")
            return False

    def RemPeriodicTask(self, key: int):
        """
        Removes an existing Periodic task from the _CAN_PERIODICTASK list and stops it. Returns True if the tasks exists
        and is removed. Returns False if task either doesn't exist or is not removed.
        :param key: key for the task in stored dictionary
        :return: Bool success
        """
        try:
            self._CAN_PERIODICTASK[key].stop()
            del self._CAN_PERIODICTASK[key]
            return True
        except Exception as error:
            self.logger.error(f"Unable to remove Periodic task, Error: {error}")
            return False
    def Subscribe(self, COBID: int, callback: callable):
        """
        Adds a CANOpen subscription to a specified COBID, the return value is handled through the specified callback
        :param COBID: COBID of the intended target
        :param callback: Callback function, should contain: (COBID, data, percentage), where percentage corresponds to the amount of the poackage received
        :return: None
        """
        try:
            self.CAN_NETWORK.subscribe(COBID, callback)
            self._CAN_SUBSCRIPTION.append(COBID)
            return True
        except Exception as error:
            self.logger.error(f"Unable to subscribe to {COBID}, Error: {error}")
            return False
    def Unsubscribe(self, COBID: int, callback: callable = None):
        """
        Removes a CANOpen subscription to a specific COBID
        :param COBID: COBID of the intended target
        :param callback: Callback function or None
        :return:
        """
        try:
            if callback == None:
                self.CAN_NETWORK.unsubscribe(COBID)
                self._CAN_SUBSCRIPTION.remove(COBID)
                self.logger.debug(f"Added subscription to {COBID}")
                return True
            else:
                self.CAN_NETWORK.unsubscribe(COBID, callback)
                self._CAN_SUBSCRIPTION.remove(COBID)
                self.logger.debug(f"Added subscription to {COBID} with callback {callback.__name__}")
                return True
        except Exception as e:
            self.logger.error(f"Error unsubscribing from {COBID}, error: {e}")
            return False

    def SDOwrite(self, node_id: int, indices: list, data: list):
        """
        Write data to a specified SDO for the given node.
        :param node_id: The ID of the node to write to.
        :param indices: A list of the index and subindex of the SDO as in [index bit 0, index bit 1, subindex bit].
        :param data: The data to be sent (as a list of hexadecimals or ints).
        :return: Bool: True if the write operation was successful, False otherwise.
        """
        data = list(data)
        success = None
        try:
            self.CAN_NODES[node_id].sdo.download(indices[0], indices[1], bytearray(data))
            success = True
        except Exception as e:
            self.logger.error(f"Error writing to SDO {indices[0]}:{indices[1]} in node {node_id}, error: {e}")
            success = False
        finally:
            return success

    def SDOread(self, node_id: int, indices: list):
        """
        Retrieve the current data from the specified SDO for the given node.
        :param node_id: The ID of the node to read from.
        :param indices: A list of the index and subindex of the sdo as in [index bit 0, index bit 1, subindex bit], e.g. [0x60, 0x60, 0x00] for index 0x6060 subindex 0x00
        return: Returns data with callback with node id 0x580 + nodeid or None if an error occurs.
        """
        data = None
        try:
            data = self.CAN_NODES[node_id].sdo.upload(indices[0], indices[1])
        except canopen.SdoCommunicationError as e:
            self.logger.error(f"Error writing to SDO {indices[0]}:{indices[1]} in node {node_id}, error: {e}")
            data = None
        finally:
            return data

    def PDOwrite(self, COBID: int, data: list):
        """
        Write data to a mapped RPDO object (Controlword)
        :param COBID: The mapped ID of the RPDO
        :param data: The data to be written (list of hexadecimals or ints).
        :return: Bool: True if successful, False if an error occurred.

        TODO: Adjust to work with nodes
        TODO: If fix is found please adapt it
        NOTE: Does not work
        """
        data = list(data)
        if len(data) <= 8:
            data += [0] * (8 - len(data))
            written_data = bytearray(data)
            try:
                self.CAN_NETWORK.send_message(COBID, written_data)
                return True
            except canopen.SdoCommunicationError as e:
                self.logger.error(f"PDO Write Error: {e}")
                return False
        else:
            self.logger.error(f"Message length over 8 bits is not supported")
            return False
    def PDOread(self, node_id: int, index: int, subindex: int = 0, timeout: int = 1, timeout_count: int = 5):
        """
        Read data from an SDO on the specified CANopen node.
        :param node_id: The ID of the node to read from.
        :param index: The index in the object dictionary to read from.
        :param subindex: The subindex in the object dictionary.
        :param timeout: Timeout in seconds for the operation.
        :param timeout_count: Amount of tries to read
        :return: Data read from the node, or raises an exception on failure.
        TODO: Does not work, please update if fix is found
        """
        timeout_counter = 0
        try:
            node = self.CAN_NODES[node_id - 1]
            # Read the data from the specified index and subindex using SDO
            data = node.sdo[index][subindex].read(timeout=timeout)
            return data
        except canopen.SdoCommunicationError as e:
            self.logger.error(f"SDO Read Error: {e}")
            raise
        except TimeoutError:
            self.logger.error(f"SDO read timed out after {timeout} seconds")
            while timeout_counter < timeout_count:
                try:
                    node = self.CAN_NODES[node_id - 1]
                    data = node.sdo[index][subindex].read(timeout=timeout)
                    timeout_counter += 1
                    return data
                except TimeoutError:
                    self.logger.error(f"SDO read timed out after {timeout} seconds")
            raise

    def NMTwrite(self, ctrlWord: list):
        ctrlWord = list(ctrlWord)
        try:
            self.CAN_NETWORK.nmt.send_command(ctrlWord[0])
            return True
        except Exception as e:
            self.logger.error(f"NMT Write Error: {e}")
            return False
