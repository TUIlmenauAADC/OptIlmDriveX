/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include "OptIlm_DriveModelTesterWidget.h"

bool initial_flag = false;

cCarControllerWidget::cCarControllerWidget(QWidget *parent) : QWidget(parent), m_ui(new Ui_CarControllerUi)
{
    m_ui->setupUi(this);

    // Rearrange buttons id 0...5 for lights
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(11), 0); // Head
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(10), 1); // Brake
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(13), 2); // Reverse
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(14), 3); // Hazard
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(12), 4); // TurnLeft
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(9), 5); // TurnRight

    //Rearrange buttons id 0...5 for drive model
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(3), 6); // Stop
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(4), 7); // Lane Keeping
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(1), 8); // TurnLeft
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(8), 9); // TurnRight
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(5), 10); // Straight
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(2), 11); // Parking
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(0), 12); // Pull out left
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(6), 13); // Pull out right
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(7), 14); // Merge left

    
//    m_timer.setInterval(50);
//    connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
//    m_timer.start();
}


cCarControllerWidget::~cCarControllerWidget()
{
    delete m_ui;
}


QButtonGroup* cCarControllerWidget::getLightButtonGroup()
{
    return m_ui->buttonGroup_lights;
}

void cCarControllerWidget::update()
{
    if(initial_flag == false)
    {
        m_ui->pushButton_stop->click();
        initial_flag = true;
    }
}
