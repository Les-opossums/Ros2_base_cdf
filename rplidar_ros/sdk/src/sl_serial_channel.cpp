/*
 * Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
 /*
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"
#include <unistd.h> // usleep -- ce fichier n'est compile que pour la cible Linux dans ce workspace


namespace sl {

    class SerialPortChannel : public ISerialPortChannel
    {
    public:
        SerialPortChannel(const std::string& device, int baudrate) :_rxtxSerial(rp::hal::serial_rxtx::CreateRxTx())
        {
            _device = device;
            _baudrate = baudrate;
        }

        ~SerialPortChannel()
        {
            if (_rxtxSerial)
                delete _rxtxSerial;
        }

        bool bind(const std::string& device, sl_s32 baudrate)
        {
            _closePending = false;
            return _rxtxSerial->bind(device.c_str(), baudrate);
        }

        bool open()
        {
            if(!bind(_device, _baudrate))
                return false;
            return _rxtxSerial->open();
        }

        void close()
        {
            _closePending = true;
            _rxtxSerial->cancelOperation();
            _rxtxSerial->close();
        }
        void flush()
        {
            _rxtxSerial->flush(0);
        }

        sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs)
        {
            _word_size_t result;
            size_t size_holder;
            size_hint = 0;

            if (_closePending) return  RESULT_OPERATION_TIMEOUT;

            if (!_rxtxSerial->isOpened()) {
                return RESULT_OPERATION_FAIL;
            }

            // IMPORTANT (perf) : avec un flux serie continu (scan haute
            // densite, ex: mode Sensitivity du A2M12), attendre ne serait-ce
            // qu'1 octet fait revenir cet appel quasi instantanement a
            // chaque fois puisqu'il y a (presque) toujours au moins 1 octet
            // deja arrive -- le thread de reception (_proc_rxThread) tourne
            // alors en quasi busy-loop : un new/delete + un lock + un
            // read()/ioctl() par octet ou presque, des milliers de fois par
            // seconde, rien que pour "attendre" un evenement qui n'en est
            // plus un.
            //
            // ATTENTION : select()/poll() sont "level-triggered" sur la
            // presence de donnees, pas sur un NOMBRE d'octets -- demander
            // directement un data_count eleve a waitfordata() ne fonctionne
            // pas : des qu'1 octet est present, select() ne bloque plus du
            // tout, et la boucle interne de waitfordata() tourne alors en
            // pur busy-spin (ioctl(FIONREAD) en rafale) jusqu'a ce que le
            // seuil soit atteint -- c'est PIRE que l'origine (verifie en
            // prod : un thread a 99% au lieu de deux a ~40%/~20%).
            //
            // La bonne approche : attendre qu'il y ait AU MOINS 1 octet
            // (bloquant, pas de busy-loop), puis faire une courte pause
            // fixe pour laisser un lot s'accumuler avant de reveiller
            // _proc_rxThread. 2ms est negligeable en latence face au reste
            // de la chaine (camera, etc.) mais suffit a regrouper plusieurs
            // octets/capsules par reveil, ce qui divise d'autant le nombre
            // d'allocations/locks/decodages par seconde.
            result = _rxtxSerial->waitfordata(1, timeoutInMs, &size_holder);
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_DEV_ERR) {
                size_hint = 0;
                return RESULT_OPERATION_FAIL;
            }
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_TIMEOUT) {
                size_hint = 0;
                return RESULT_OPERATION_TIMEOUT;
            }

            static const useconds_t kCoalesceUs = 2000; // 2ms
            usleep(kCoalesceUs);

            size_t available = _rxtxSerial->rxqueue_count();
            size_hint = (available > size_holder) ? available : size_holder;

            return RESULT_OK;
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (_closePending) return false;
            return (_rxtxSerial->waitfordata(size, timeoutInMs, actualReady) == rp::hal::serial_rxtx::ANS_OK);
        }

        int write(const void* data, size_t size)
        {
           return _rxtxSerial->senddata((const sl_u8 * )data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t lenRec = 0;
            lenRec = _rxtxSerial->recvdata((sl_u8 *)buffer, size);
            return (int)lenRec;
        }

        void clearReadCache()
        {

        }

        void setDTR(bool dtr)
        {
            dtr ? _rxtxSerial->setDTR() : _rxtxSerial->clearDTR();
        }

        int getChannelType() {
            return CHANNEL_TYPE_SERIALPORT;
        }

    private:
        rp::hal::serial_rxtx  * _rxtxSerial;
        bool _closePending;
        std::string _device;
        int _baudrate;

    };

    Result<IChannel*> createSerialPortChannel(const std::string& device, int baudrate)
    {
        return new  SerialPortChannel(device, baudrate);
    }

}
