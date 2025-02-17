# 프로그램 및 라이브러리 설치

1. 먼저 전체 파일을 다운로드 받습니다. (이하, Windows 사용자 기준으로 설명)

2. Arduino Software를 설치합니다.

    https://www.arduino.cc/en/software<br/>
    1.8.x 버전 이상

3. 사용되는 Arduino Library를 설치합니다.

    [BlackberryEdu\설치용-프로그램-드라이버\arduino-libraries]

4. PixyMon 프로그램을 설치합니다.

    pixymon_v2_windows-3.0.24.exe

# Blackberry와 Arduino 연동

1. Arduino 프로그램 실행

2. Blackberry와 PC를 USB 케이블로 연결

    USB 드라이버를 설치합니다. (CH340 또는 CH341 시리얼 드라이버)

    [USB-DRIVE-CH34x\CH341SER.EXE] 실행, 또는
    https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all#drivers-if-you-need-them 참고

    만약, USB 케이블을 연결하고 Blackberry 전원 스위치를 켰을 때 새로운 COM 포트가 추가되지 않으면 USB 드라이버를 다시 설치해야 합니다. (PC 장치 관리자에서 확인하거나, 간단히 [Tools] -> [Port] 메뉴에서 확인 가능함)

3. Arduino 프로그램에서 아래 자세 초기화 예제 코드를 엽니다.

    [File] -> [Open]

    "BlackberryEdu\sourceCode\MobileManipulator_Blackberry_00_00_init_pos\MobileManipulator_Blackberry_00_00_init_pos.ino"

4. Blackberry에 맞는 Board와 Port를 선택합니다.

    [Tools] -> [Board] -> [Arduino AVR Boards] -> [Arduino Mega or Mega 2560]
    [Tools] -> [Port] -> COM# (Arduino Mega ADK)

5. Blackberry에 Upload 합니다.

    [Sketch] -> [Upload, Ctrl+U] 선택, 또는 <br/>
    툴 바에서 '오른쪽 화살표 키(->)' 아이콘 클릭
