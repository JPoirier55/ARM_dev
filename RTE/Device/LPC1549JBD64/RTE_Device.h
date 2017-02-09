/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2016 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * $Date:        14. June 2016
 * $Revision:    V1.1.0
 *
 * Project:      RTE Device Configuration for NXP LPC15xx
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H


// <e> USB Controller
#define RTE_USB_USB0                   0

//   <e> Pin Configuration
#define RTE_USB_USB_PIN                1
//     <e> USB_VBUS
//     <i> USB VBUS signal
//     <i> USB VBUS function assignment
//     <i> PINx_y
//       <o1> Pin  <0=>P0_16 <1=>P1_11 <2=>P2_4
//     </e>
#define RTE_USB_VBUS_PIN               1
#define RTE_USB_VBUS_ID                2
#if (RTE_USB_VBUS_ID == 0)
  #define RTE_USB_VBUS_PORT            0
  #define RTE_USB_VBUS_BIT             16
#elif (RTE_USB_VBUS_ID == 1)
  #define RTE_USB_VBUS_PORT            1
  #define RTE_USB_VBUS_BIT             11
#elif (RTE_USB_VBUS_ID == 2)
  #define RTE_USB_VBUS_PORT            2
  #define RTE_USB_VBUS_BIT             4
#else
  #error "Invalid RTE_USB_VBUS Pin Configuration!"
#endif
//   </e>
// </e>

// <e> SPI0 (Serial Peripheral Interface 0) [Driver_SPI0]
// <i> Configuration settings for Driver_SPI0 in component ::Drivers:SPI
#define RTE_SPI0                        0

//   <h> SPI0_SCK Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </h>
#define RTE_SPI0_SCK_PIN                1
#define RTE_SPI0_SCK_PORT               0
#define RTE_SPI0_SCK_BIT                0


//   <e> SPI0_SSEL Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI0_SSEL_PIN               0
#define RTE_SPI0_SSEL_PORT              0
#define RTE_SPI0_SSEL_BIT               0

//   <e> SPI0_MOSI Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI0_MOSI_PIN               0
#define RTE_SPI0_MOSI_PORT              0
#define RTE_SPI0_MOSI_BIT               0

//   <e> SPI0_MISO Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI0_MISO_PIN               0
#define RTE_SPI0_MISO_PORT              0
#define RTE_SPI0_MISO_BIT               0

// </e>

// <e> SPI1 (Serial Peripheral Interface 1) [Driver_SPI1]
// <i> Configuration settings for Driver_SPI1 in component ::Drivers:SPI
#define RTE_SPI1                        0

//   <h> SPI1_SCK Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </h>
#define RTE_SPI1_SCK_PIN                1
#define RTE_SPI1_SCK_PORT               0
#define RTE_SPI1_SCK_BIT                0

//   <e> SPI1_SSEL Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI1_SSEL_PIN               0
#define RTE_SPI1_SSEL_PORT              0
#define RTE_SPI1_SSEL_BIT               0

//   <e> SPI1_MOSI Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI1_MOSI_PIN               0
#define RTE_SPI1_MOSI_PORT              0
#define RTE_SPI1_MOSI_BIT               0

//   <e> SPI1_MISO Pin
//   <i> Configure Pin if exists
//   <i> GPIO PIOxy (x = 1..2, y = 0..31)
//     <o1> Port <0=>PIO0 <1=>PIO1 <2=>PIO2
//     <i>  Selects Port Name
//     <o2> Bit <0-32>
//     <i>  Selects Port Bit
//   </e>
#define RTE_SPI1_MISO_PIN               0
#define RTE_SPI1_MISO_PORT              0
#define RTE_SPI1_MISO_BIT               0

// </e>

#endif  /* __RTE_DEVICE_H */
