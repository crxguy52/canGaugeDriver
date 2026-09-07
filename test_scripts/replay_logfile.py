import can, logging, os
from typing import Iterable, cast
from datetime import datetime   

logging.root.setLevel(logging.INFO)

if __name__ == '__main__':
    printDebug = 0
    playbackFilePath = 'test_scripts/GMLAN_2026-02-14_14-39-30.blf'
    logging.info(f'Playing back {playbackFilePath}')

    # Create a virtual bus using the same interface
    logging.info('Starting bus')
    with can.Bus(interface='socketcan', channel='can0', baud=500e3) as bus:

        logging.info(f'Opening LogReader {playbackFilePath}')
        with can.LogReader(playbackFilePath) as reader:

            logging.info('Creating sync')
            in_sync = can.MessageSync(cast(Iterable[can.Message], reader), skip=5)

            logging.info(f"Can LogReader (Started on {datetime.now()})")

            for message in in_sync:
                if message.is_error_frame:
                    continue
                if printDebug:
                    logging.info(message)
                bus.send(message)
      