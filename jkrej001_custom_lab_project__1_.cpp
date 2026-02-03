#include <avr/io.h>
#include <util/delay.h>
#include <helper.h>
#include <periph.h>
#include <serialATmega.h>
#include <spiAVR.h>
#include <timerISR.h>

// Display dimensions
const int WIDTH   = 128;
const int HEIGHT  = 128;

// Command definitions
const int NOP    = 0x00;
const int SWRESET= 0x01;
const int SLPIN  = 0x10;
const int SLPOUT = 0x11;
const int DISPOFF= 0x28;
const int DISPON = 0x29;
const int CASET  = 0x2A;
const int RASET  = 0x2B;
const int RAMWR  = 0x2C;
const int MADCTL = 0x36;
const int COLMOD = 0x3A;

// Color definitions in BGR format
const int BLACK =  0x0000;
const int RED   =  0x001F;
const int GREEN =  0x07E0;
const int BLUE  =  0xF800;
const int WHITE =  0xFFFF;

// Pin definitions for PORTB
const int RST  =   0;   // Reset pin (B0)
const int DC   =   1;   // Data/Command pin (B1)
const int CS   =   2;   // Chip Select pin (B2)
const int MOSI =   3;   // SPI MOSI pin (B3)
const int SCK  =   5;   // SPI Clock pin (B5)

bool crown[5][9] = {
    {1,0,0,0,1,0,0,0,1},
    {1,1,0,1,1,1,0,1,1},
    {1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1}
};

int piece[14][14] = {
    {0,0,0,0,0,2,2,2,2,0,0,0,0,0},
    {0,0,0,2,2,1,1,1,1,2,2,0,0,0},
    {0,0,2,1,1,1,1,1,1,1,1,2,0,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {2,1,1,1,1,1,1,1,1,1,1,1,1,2},
    {2,1,1,1,1,1,1,1,1,1,1,1,1,2},
    {2,1,1,1,1,1,1,1,1,1,1,1,1,2},
    {2,1,1,1,1,1,1,1,1,1,1,1,1,2},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,0,2,1,1,1,1,1,1,1,1,2,0,0},
    {0,0,0,2,2,1,1,1,1,2,2,0,0,0},
    {0,0,0,0,0,2,2,2,2,0,0,0,0,0}
};

int king[14][14] = {
    {0,0,0,0,0,2,2,2,2,0,0,0,0,0},
    {0,0,0,2,1,1,1,1,1,1,2,0,0,0},
    {0,0,2,1,1,1,1,1,1,1,1,2,0,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {2,1,1,2,1,1,2,2,1,1,2,1,1,2},
    {2,1,1,2,2,1,2,2,1,2,2,1,1,2},
    {2,1,1,2,2,2,2,2,2,2,2,1,1,2},
    {2,1,1,2,2,2,2,2,2,2,2,1,1,2},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,2,1,1,1,1,1,1,1,1,1,1,2,0},
    {0,0,2,1,1,1,1,1,1,1,1,2,0,0},
    {0,0,0,2,1,1,1,1,1,1,2,0,0,0},
    {0,0,0,0,0,2,2,2,2,0,0,0,0,0}
};

int pieces[8][8] = {
    {1,0,1,0,1,0,1,0},
    {0,1,0,1,0,1,0,1},
    {1,0,1,0,3,0,1,0},
    {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0},
    {0,2,0,4,0,2,0,2},
    {2,0,2,0,2,0,2,0},
    {0,2,0,2,0,2,0,2}
};

bool black_turn = 0;

// Helper function to set/clear specific bits
void writePin(volatile uint8_t *port, uint8_t pin, uint8_t value) {
    if (value) {
        *port |= (1 << pin);
    } else {
        *port &= ~(1 << pin);
    }
}

// Initialize SPI in master mode
void SPI_Init() {
    // Set MOSI, SCK, and CS as outputs
    DDRB |= (1 << MOSI) | (1 << SCK) | (1 << CS);
    
    // Enable SPI, set as master, and set clock rate (fosc/4)
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = (1 << SPI2X);  // Double SPI speed
}

// Send byte via SPI
void SPI_Write(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));  // Wait for transmission complete
}

// Send command to display
void writeCommand(uint8_t cmd) {
    writePin(&PORTB, DC, 0);  // Command mode
    writePin(&PORTB, CS, 0);  // Select display
    SPI_Write(cmd);
    writePin(&PORTB, CS, 1);  // Deselect display
}

// Send data to display
void writeData(uint8_t data) {
    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display
    SPI_Write(data);
    writePin(&PORTB, CS, 1);  // Deselect display
}

// Initialize display
void InitDisplay() {
    // Set control pins as outputs
    DDRB |= (1 << RST) | (1 << DC) | (1 << CS);
    
    // Hardware reset
    writePin(&PORTB, RST, 0);
    _delay_ms(100);
    writePin(&PORTB, RST, 1);
    _delay_ms(100);

    // Software reset
    writeCommand(SWRESET);
    _delay_ms(150);
    
    // Exit sleep mode
    writeCommand(SLPOUT);
    _delay_ms(500);
    
    // Set color mode to 16-bit per pixel
    writeCommand(COLMOD);
    writeData(0x05);
    _delay_ms(10);
    
    // Set memory access control
    writeCommand(MADCTL);
    writeData(0xC0);  // Row/column address, RGB color order
    
    // Turn on the display
    writeCommand(DISPON);
    _delay_ms(100);
}

// Set the drawing window
void setAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    writeCommand(CASET);  // Column address set
    writeData(0x00);
    writeData(x0);              // Start column
    writeData(0x00);
    writeData(x1);              // End column

    writeCommand(RASET);  // Row address set
    writeData(0x00);
    writeData(y0);              // Start row
    writeData(0x00);
    writeData(y1);              // End row

    writeCommand(RAMWR);  // Memory write
}

void setPixel(uint16_t x, uint16_t y, uint16_t color) {
    // Ensure the pixel is within display bounds
    if (x >= WIDTH || y >= HEIGHT + 5) { // needed to fill entire screen
        return; // Out of bounds
    }

    // Set the address window to the specific pixel
    setAddressWindow(x, y, x, y);

    // Send the pixel color
    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display

    SPI_Write(color >> 8);    // High byte of color
    SPI_Write(color & 0xFF);  // Low byte of color

    writePin(&PORTB, CS, 1);  // Deselect display
}

// Fill screen with a single color
void fillScreen(uint16_t color) {
    // Set the drawing window to entire display
    setAddressWindow(0, 0, WIDTH - 1, HEIGHT - 1);
    
    // Calculate high and low bytes of color
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;
    
    // Write color to every pixel
    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display
    
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        SPI_Write(high);
        SPI_Write(low);
    }
    
    writePin(&PORTB, CS, 1);  // Deselect display
}

void drawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) { //bottom left corner is passed in as x,y
    setAddressWindow(x, y, x + w, y + h);

    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;

    writePin(&PORTB, DC, 1);
    writePin(&PORTB, CS, 0);

    for(uint16_t i = 0; i < w * h; i++) {
        SPI_Write(high);
        SPI_Write(low);
    }

    writePin(&PORTB, CS, 1);  // Deselect display    
}

void checkerBoard(uint16_t color1, uint16_t color2) {
    int num_rows = 8;
    int num_columns = 8;
    int row_height = HEIGHT / num_rows;
    int col_width = WIDTH / num_columns;

    for(int row = 0; row < num_rows; ++row) {
        for(int column = 0; column < num_columns; ++column) {
            if ((row + column) % 2) {
                drawRectangle(column * col_width + 1, row * row_height + 2, col_width, row_height+1, color1);
            }
            else {
                drawRectangle(column * col_width + 1, row * row_height + 2, col_width, row_height+1, color2);
            }
        }
    }
}

void drawCircle(uint16_t x, uint16_t y, int r, uint16_t color) { //no bitmapping, pass in center, radius, color, unused in current ver
    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display

    for(int i = -r; i < 2 * r; i++) {
        for (int j = -r; j < 2 * r; j++) {
            if (i * i + j * j <= r * r) {
                setPixel( x + i, j + y, WHITE); //outer circle for outline
            }
            if (i * i + j * j <= (r * r) - 2 * r) {
                setPixel( x + i, j + y, color); //inner circle for main color
            }
        }
    }

    writePin(&PORTB, CS, 1);  // Deselect display
}

void drawPiece_BM(uint16_t x, uint16_t y, uint16_t color) { //bitmapping, pass in corner & color, fixed size
    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display

    for(int i = 0; i < 14; i++) {
        for (int j = 0; j < 14; j++) {
            if (piece[i][j] == 2) {
                setPixel( x + i + 2, j + y + 3, WHITE); //outer circle for outline
            }
            if (piece[i][j] == 1) {
                setPixel( x + i + 2, j + y + 3, color); //inner circle for main color
            }
        }
    }
}

void drawKing_BM(uint16_t x, uint16_t y, uint16_t color) {
    //serial_println("Printing King");
    setAddressWindow(x,y,x+14,y+14);

    writePin(&PORTB, DC, 1);  // Data mode
    writePin(&PORTB, CS, 0);  // Select display

    for(int i = 0; i < 14; i++) {
        for (int j = 0; j < 14; j++) {
            if (king[j][i] == 2) {
                setPixel( x + i + 2, j + y + 3, WHITE); //outer circle for outline & crown
            }
            if (king[j][i] == 1) {
                setPixel( x + i + 2, j + y + 3, color); //inner circle for main color
            }
        }
    }
}

void drawSquareOutline(uint16_t x, uint16_t y, uint16_t size, uint16_t color) {
    // Top side
    drawRectangle(x, y, size, 2, color);
    // Bottom side
    drawRectangle(x, y + size, size, 2, color);
    // Left side
    drawRectangle(x, y, 2, size+5, color);
    // Right side
    drawRectangle(x + size, y, 2, size+5, color); //random modifications to numbers to make it render better
}

void drawCrown(uint16_t x, uint16_t y) { //unused, replaced with drawKing_BM
    setAddressWindow(x - 5, y - 2, x + 4, y + 2); 
    x = x - 5;                                    
    y = y - 2;                                    
                                                  
    writePin(&PORTB, DC, 1);  // Data mode          
    writePin(&PORTB, CS, 0);  // Select display     

    for(int i = x; i < 9; ++i) {
        for(int j = y; j < 5; ++j) {
            if(crown[y][x]) {
                setPixel( x + i, j + y, WHITE);
            }
        }
    }
}

int ADC_out(){ // 
  int x = ADC_read(3);
  int y = ADC_read(2);
  if (y >= 768 && y >= x && y-512 >= 512-x) { //down
    return 3;
  }
  else if (y <= 256 && y <= x && 512-y >= x-512) { //up
    return 4;
  }
  else if (x >= 768 && x > y && x-512 > 512-y) { //left
    return 2;
  }
  else if (x <= 256 && x < y && 512-x > y-512) { //right
    return 1;
  }
  else{
    return 0;
  }
}

bool isMoveValid(int startX, int startY, int endX, int endY) {
    if (startX < 0 || startX >= 8 || startY < 0 || startY >= 8 ||
        endX < 0 || endX >= 8 || endY < 0 || endY >= 8) {
        return false; // Out of bounds
    }

    int piece = pieces[startY][startX];
    if (piece == 0) {
        return false; // No piece to move
    }

    // Check if the end position is occupied
    if (pieces[endY][endX] != 0) {
        return false; // Cannot move to an occupied square
    }

    // Determine movement direction and type of piece (king or regular)
    bool isKing = (piece == 2); // Assuming 2 represents a king piece
    int direction = (piece == 1) ? 1 : -1; // Regular pieces move forward

    // Check for simple moves (diagonal step)
    if ((endX == startX + 1 || endX == startX - 1) &&
        (endY == startY + direction || (isKing && endY == startY - direction))) {
        return true; // Valid simple move
    }

    //captures checked in another function.
    return false;
}




/*void performCapture(int startRow, int startCol, int endRow, int endCol) {

}*/

bool checkForWin(int targetPieceType) { //takes 1 or 2, also captures 3s and 4s so you cant lose if you have a king
    // Check if there are any pieces of the target type left
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (pieces[row][col] == targetPieceType || pieces[row][col] == targetPieceType + 2) {
                return false; // At least one piece of the target type exists
            }
        }
    }
    return true; // No pieces of the target type remain
}


//Shared Variables

bool DS_done = 0;
int JS_input = 0;
int C_pos[2] = {0,0};
bool MM_piecechosen = 0;
int MM_ptomove[2] = {0,0};

/* TODO: match with how many tasks you have */
#define NUM_TASKS 3

//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;


//TODO: Define Periods for each task
// e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = 50;/* TODO: Calulate GCD of tasks */
const unsigned long DS_PERIOD = 500; //takes a while to run, have to give it big task period so it finishes before called again
const unsigned long JS_PERIOD = 50;
const unsigned long MM_PERIOD = 50;

task tasks[NUM_TASKS]; // declared task array with tasks

//TODO: Define, for each task:
// (1) enums and
// (2) tick functions

enum DS_states{DS_renderscreen, DS_wait};
enum JS_states{JS_neutral,JS_dir};
enum MM_states{MM_wait,MM_press,MM_move};

int TickFCT_DS(int state){
    switch(state) {
        case DS_renderscreen:
            if (DS_done) {
                state = DS_wait;
            }
            else {
            state = DS_renderscreen;
            }
            break;
        case DS_wait:
            if (DS_done) {
                state = DS_wait;
            }
            else {
            state = DS_renderscreen;
            }
            break;
    }

    switch(state) {
        case DS_renderscreen:
            //drawRectangle(64,64,16,16,GREEN);
            //fillScreen(GREEN);
            checkerBoard(RED,BLACK);
            for(int i = 0; i < 8; ++i) {
                for (int j = 0; j < 8; ++j) {
                    if (pieces[i][j] == 2) {
                        drawPiece_BM(16 * j, 16 * i, BLACK);
                    }
                    else if (pieces[i][j] == 1) {
                        drawPiece_BM(16 * j, 16 * i, RED);
                    }
                    else if (pieces[i][j] == 3) {
                        drawKing_BM(16 * j, 16 * i, RED);
                    }
                    else if (pieces[i][j] == 4) {
                        drawKing_BM(16 * j, 16 * i, BLACK);
                    }
                }
            }
            drawSquareOutline(C_pos[0] * 16, C_pos[1] * 16, 16, WHITE);
                /*serial_println("x=");
                serial_println(C_pos[0]);
                serial_println("y=");
                serial_println(C_pos[1]);*/
            DS_done = 1;
            break;
        case DS_wait:
            break;
    }

    return state;
}

int TickFCT_JS(int state) {
    switch(state) {
        case JS_neutral:
            JS_input = ADC_out();
            if(JS_input == 0) {
                state = JS_neutral;
            }
            else {
                state = JS_dir;
                if (JS_input == 3 && C_pos[1] > 0) { //up
                    C_pos[1]--;
                } else if (JS_input == 4 && C_pos[1] < 7) { //down
                    C_pos[1]++;
                } else if (JS_input == 2 && C_pos[0] < 7) { //left
                    C_pos[0]++;
                } else if (JS_input == 1 && C_pos[0] > 0) { //right
                    C_pos[0]--;
                }
                /*serial_println("C_pos[0]:");
                serial_println(C_pos[0]);
                serial_println("C_pos[1]:");
                serial_println(C_pos[1]);*/
                DS_done = 0; 
            }
            break;
        case JS_dir: 
            if(ADC_out()!=0) {
                state = JS_dir;
            }
            else {
                state = JS_neutral;
            }
            break;
    }
    switch(state) {
        case JS_neutral:
            break;
        case JS_dir:
            break;
    }

    return state;
}

int TickFCT_MM(int state) {
    switch(state) {
        case MM_wait:
            if (GetBit(PINC,4)) {
                state = MM_wait;
            }
            else if (!GetBit(PINC,4) && (pieces[C_pos[1]][C_pos[0]] != 0)) {
                MM_ptomove[0] = C_pos[0];
                MM_ptomove[1] = C_pos[1];
                state = MM_press;
            }
            else {
                state = MM_wait;
            }
            break;
        case MM_press:
            if (!GetBit(PINC,4)) {
                state = MM_press;
            }
            else if (GetBit(PINC,4) && !MM_piecechosen) {
                state = MM_move;
                MM_piecechosen = !MM_piecechosen;
            }
            else if  (GetBit(PINC,4) && MM_piecechosen) {
                state = MM_wait;
                MM_piecechosen = !MM_piecechosen;
            }
            break;
        case MM_move:
            if (!GetBit(PINC,4)) {
                if(pieces[C_pos[1]][C_pos[0]] == 0) {
                    pieces[C_pos[1]][C_pos[0]] = pieces[MM_ptomove[1]][MM_ptomove[0]];
                    pieces[MM_ptomove[1]][MM_ptomove[0]] = 0;
                    MM_ptomove[0] = 0;
                    MM_ptomove[1] = 0;
                    DS_done = 0;
                    state = MM_press;          
                }
                else {
                    state = MM_press;
                }
            }
            else {
                state = MM_move;
            }
            break;
    }
    switch(state) {
        case MM_wait:
            //serial_println(1);
            break;
        case MM_press:
            //serial_println(2);
            break;
        case MM_move:
            //serial_println(3);
            break;
    }

    return state;
}


void TimerISR() {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {                // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}

int main(void) {
    // Initialize SPI and display
    SPI_Init();
    InitDisplay();
    serial_init(9600);
    ADC_init();   // initializes ADC

    DDRC = 0x00;
    PORTC = 0xFF;
    
    //testing various outputs lol
    //fillScreen(0x00E7);
    //checkerBoard(0x00E7, 0x01EF); Dark and light brown
    //drawCircle(64,64,7,BLUE);

    //TODO: Initialize tasks here
    tasks[0].period = DS_PERIOD;
    tasks[0].state = DS_renderscreen;
    tasks[0].elapsedTime = tasks[0].period;
    tasks[0].TickFct = &TickFCT_DS;

    tasks[1].period = JS_PERIOD;
    tasks[1].state = JS_neutral;
    tasks[1].elapsedTime = tasks[1].period;
    tasks[1].TickFct = &TickFCT_JS;

    tasks[2].period = MM_PERIOD;
    tasks[2].state = MM_wait;
    tasks[2].elapsedTime = tasks[2].period;
    tasks[2].TickFct = &TickFCT_MM;

    TimerSet(GCD_PERIOD);
    TimerOn();
    
    while(1) {
        // Main loop
    }
    
    return 0;
}