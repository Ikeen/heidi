<?php
    header("Access-Control-Allow-Origin: *");
    header("Access-Control-Allow-Methods: POST, GET"); //alles wieder zu POST machen!!!!!!!!!!!!

    //Connect to database
    $servername = "localhost";
    $username   = "HeidiTracker";
    $password   = "jETh4dBShgwFawQ";
    $dbname     = "Heidi";
    $herdeID    = '';

    // Create connection
    $conn = new mysqli($servername, $username, $password, $dbname);
    // Check connection
    if ($conn->connect_errno) {
        echo "Database Connection failed: " . $conn->connect_error;
        die("Database Connection failed: " . $conn->connect_error);
    }
    if(!empty($_GET["X01"]) || !empty($_POST["X01"])){
      //php.ini: max_input_var = 1000 by default; post_max_size = 8M by default
      $stateOK    = TRUE;
      $i          = 1;
      $rounds     = 0; //prevent from running wild
      while(($i > 0) && ($rounds < 100) && $stateOK){
        if ($i < 10){ $urlmarker = "X0{$i}"; } else { $urlmarker = "X{$i}"; }
        if (empty($_POST[$urlmarker]) && empty($_GET[$urlmarker])){ break; }
        $rounds++;
        if (empty($_POST[$urlmarker])) { $urldata = $_GET[$urlmarker]; }
        else { $urldata = $_POST[$urlmarker]; }
        //echo  $urldata . '<br>';
        $b64 = strtr($urldata, '-_', '+/');
        //echo $b64 . "<br>";
        $dec = base64_decode($b64, false);
        if(!$dec) { continue; }
        //echo bin2hex($dec) . "<br>";
        $data = unpack("C*", $dec);
        $pos = 1;
        $count = $data[$pos++];
        //echo "<br>" . $count . ", " . $pos . "<br>";
        if($count < 7 ) { continue; }
        //uint_8
        $herde = $data[$pos++];
        if($herde == 0) { continue; }
        if ($herde < 10) { $herde = '000' . $herde; }
        elseif ($herde < 100) { $herde = '00' . $herde; }
        elseif ($herde < 1000) { $herde = '0' . $herde; }
        if ($herdeID == '') { $herdeID = $herde; }
        //echo $herde . ", " . $pos . "<br>";
        //uint_8
        $animal = $data[$pos++];
        if($animal == 0) { continue; }
        if ($animal < 10) { $animal = '000' . $animal; }
        elseif ($animal < 100) { $animal = '00' . $animal; }
        elseif ($animal < 1000) { $animal = '0' . $animal; } 
        $id = $herde . '.' . $animal;
        //echo $animal . ", " . $pos . "<br>";
        //int_32 / 1000000
        $lat = (integer)($data[$pos++] + ($data[$pos++] << 8) + ($data[$pos++] << 16) + ($data[$pos++] << 24));
        $lat /= 1000000;
        if($lat > 90) { continue; }
        if($lat < -90) { continue; }
        //echo  $lat . ", " . $pos . "<br>";
        //int_32 / 1000000
        $lng = (integer)($data[$pos++] + ($data[$pos++] << 8) + ($data[$pos++] << 16) + ($data[$pos++] << 24));
        $lng /= 1000000;
        if($lng > 180) { continue; }
        if($lng < -180) { continue; }
        //echo  $lng . ", " . $pos . "<br>";
        //int_16 (needs special threadmet)
        if($data[$pos+1] > 127)
        { $alt = (integer)($data[$pos++] + ($data[$pos++] << 8) + (0xFF << 16) + (0xFF << 24)); }
        else
        { $alt = (integer)($data[$pos++] + ($data[$pos++] << 8)); }
        if($alt > 10000) { continue; }
        if($alt < -100) { continue; }
        //echo  $alt . ", " . $pos . "<br>";
        //16-bit DOS-date (may base on year 2020)
        $_date = $data[$pos++] + ($data[$pos++] << 8);
        $year = ($_date >> 9) + 1980;
        $mon  = ($_date >> 5) & 0x0F;
        if ($mon < 10) { $mon = '0' . $mon; }
        $day  =  $_date  & 0x1F;
        if ($day < 10) { $day = '0' . $day; }
        $date = $year . '-' . $mon . '-' . $day;
        //echo  $date . ", " . $pos . "<br>";
        //16-bit DOS-time
        $_time = $data[$pos++] + ($data[$pos++] << 8);
        $hour = ($_time >> 11);
        if ($hour < 10) { $hour = '0' . $hour; } 
        $min  = ($_time >> 5)  & 0x3F;
        if ($min < 10) { $min = '0' . $min; } 
        $sec  = ($_time  & 0x1F) << 1;
        if ($sec < 10) { $sec = '0' . $sec; }
        $time = $hour . ':' . $min . ':' . $sec;
        //echo  $time . ", " . $pos . "<br>";
        if (!validateDate($date . " " . $time)){ $i++; continue; }
        //uint_16 / 1000
        $batt = (integer)($data[$pos++] + ($data[$pos++] << 8));
        $batt /= 1000;
        if ($batt > 9){ $i++; continue; } //check database value boundaries
        //uint_16 / 100
        if($data[$pos+1] > 127)
        { $temp = (integer)($data[$pos++] + ($data[$pos++] << 8) + (0xFF << 16) + (0xFF << 24)); }
        else
        { $temp = (integer)($data[$pos++] + ($data[$pos++] << 8)); }
        //$temp /= 100;
        if (($temp > 999) || ($temp < -999)){ $i++; continue; } //check database value boundaries
        //echo  "temp: " . $temp . ", " . $pos . "<br>";
        $err = "0x" . strtoupper(dechex((integer)($data[$pos++] + ($data[$pos++] << 8))));
        //echo  "errc: " . $err . ", " . $pos . "<br>";
        $sql0 = "SELECT Count(*) FROM TrackerData WHERE TrackerID = '{$id}' AND TimeStamp = '{$date} {$time}';";
        $sql1 = "INSERT INTO TrackerData (TrackerID, Longitude, Latitude, Altitude, TimeStamp, Battery, Temperature, ErrorCode ";
        $sql2 = "VALUES ('{$id}', '{$lng}', '{$lat}', '{$alt}', '{$date} {$time}', '{$batt}', '{$temp}', '{$err}'";
        //now add all other parameters
        for ($x = 1; $x <= $count-9; $x++) {
          //uint_16
          $sql1 .= ", FreeValue{$x}";
          $free = (integer)($data[$pos++] + ($data[$pos++] << 8));
          //echo  'FreeValue ' . $x . ': ' . $free . ", " . $pos . "<br>";
          $sql2 .= ", '{$free}'";
        }
        //check, if the data set is already in
        if ($sqlres = $conn->query($sql0)){
          if ($row = $sqlres->fetch_row()){
            if ($row[0] == 0){ //no? then push it in
              $sql = "{$sql1}) {$sql2})";
              $stateOK = $conn->query($sql);
            } else { $stateOK = TRUE; }
            $i++;
          } else { $i = 0; }
          $sqlres->close();
        } else {
          echo "sql error: " . $conn->error . "<br>";
          $i = 0;
          $stateOK = FALSE;
        }
      }
      $i=1;
      //$fence_data = array():
      if ($stateOK === TRUE) {
        echo "OK";
        //tellFence($conn, $herdeID);
        //tellSettings($conn, $herdeID);
        //tellTelNo($conn, $herdeID);
      } else {
        echo "Error: " . $sql . "<br>" . $conn->error;
      }
    } else {
      echo "OK";
      if(!empty($_POST["ID"])){
        $herdeID = substr($_POST["ID"],0,4);
        tellFence($conn, $herdeID);
        tellSettings($conn, $herdeID);
        tellTelNo($conn, $herdeID);
      } else if (isset($_GET['ID'])) {
        $herdeID = substr($_GET['ID'],0,4);
        tellFence($conn, $herdeID);
        tellSettings($conn, $herdeID);
        tellTelNo($conn, $herdeID);
      }
      else { echo " - no valid Data"; }
    }
    $conn->close();

function tellFence($conn, $herdeID)
{
  $sql0 = "SELECT Longitude, Latitude FROM Fence WHERE HerdeID = '{$herdeID}' AND Active = 1;";
  //error_log($sql0);
  if ($sqlres = $conn->query($sql0)){ 
    $fence_data = pack('C', (integer)$sqlres->num_rows);
    $i = 0;
    while ($row = $sqlres->fetch_row()){
      $fence_data .= pack('i', (integer)($row[0]*1000000)); 
      $fence_data .= pack('i', (integer)($row[1]*1000000));
      $i++;
    }
    //error_log("poles: " . $i);
    $sqlres->close();
    $b64n = base64_encode($fence_data);
    $b64n = strtr($b64n, '+/', '-_');
    $b64n = preg_replace("/[^a-zA-Z0-9-_]/", "", $b64n);
    echo ';' . $b64n . ';' . dechex(crc16F($b64n));
  }
}

function tellSettings($conn, $herdeID)
{
  $sql0 = "SELECT CyclesDay, CyclesNight, CycleLenDay, CycleLenNight, NightStartHour, NightEndHour, DistAlarmThr, AccThreshold1, AccThr1AlarmCnt, AccThreshold2, AccThr2AlarmCnt, accNightFactor FROM Settings WHERE HerdeID = '{$herdeID}';";
  if ($sqlres = $conn->query($sql0)){ 
    if ($row = $sqlres->fetch_row()){ $count = $sqlres->field_count; } else { $count = 0; }
    $settings_data = pack('C', (integer)$count);
    $i = 0; 
    while ($i < $count){
      $settings_data .= pack('S', (integer)($row[$i])); 
      $i++;
    }
    $sqlres->close();
    $b64n = base64_encode($settings_data);
    $b64n = strtr($b64n, '+/', '-_');
    $b64n = preg_replace("/[^a-zA-Z0-9-_]/", "", $b64n);
    echo ';' . $b64n . ';' . dechex(crc16F($b64n));
  }
}
function tellTelNo($conn, $herdeID)
{
  $sql0 = "SELECT TelNo1, TelNo2 FROM Settings WHERE HerdeID = '{$herdeID}';";
  if ($sqlres = $conn->query($sql0)){ 
    $tel1 = '';
    $tel2 = '';
    $count = 0;
    if ($row = $sqlres->fetch_row()){
      $count = $sqlres->field_count;
      $tel1 = $row[0];
      if ($count >= 2) { $tel2 = $row[1]; } 
    }
    if($tel2 == '') {$count = 1;} 
    if($tel1 == '') {$count = 0;} 
    $settings_data = pack('C', (integer)$count);
    if  ($count >= 1) {
      for ($i = 0; $i < 12; $i++){
        if (strlen($tel1) >= ($i*2+1)){
          $s1 = substr($tel1, $i*2, 1);
          if ($s1 != ' ') { $a = intval($s1); }
          else { $a = 11; }
        } else { $a = 11; }
        if (strlen($tel1) >= ($i*2+2)){
          $s2 = substr($tel1, $i*2+1, 1);
          if ($s2 != ' ') { $b = intval($s2); }
          else { $b = 11; }
        } else { $b = 11; }
        $x = $a | ($b << 4);
        $settings_data .= pack('C',(integer)($x));
      }
    }
    if  ($count >= 2) {
      for ($i = 0; $i < 12; $i++){
         if (strlen($tel2) >= ($i*2+1)){
           $s1 = substr($tel2, $i*2, 1);
           if ($s1 != ' ') { $a = intval($s1); }
           else { $a = 11; }
         } else { $a = 11; }
         if (strlen($tel2) >= ($i*2+2)){
           $s2 = substr($tel2, $i*2+1, 1);
           if ($s2 != ' ') { $b = intval($s2); }
           else { $b = 11; }
         } else { $b = 11; }
        $x = $a | ($b << 4);
        $settings_data .= pack('C',(integer)($x));
      }
    }
    if ($count > 0) {
      $b64n = base64_encode($settings_data);
      $b64n = strtr($b64n, '+/', '-_');
      $b64n = preg_replace("/[^a-zA-Z0-9-_]/", "", $b64n);
      echo ';' . $b64n . ';' . dechex(crc16F($b64n));
    }
    $sqlres->close();
  }
}

function crc16F($data) //CRC-16/CCITT-FALSE
{
  $crc = 0xFFFF;
  $len = strlen($data);
  for ($i = 0; $i < $len; $i++)
  {
    $x = (($crc >> 8) ^ ord($data[$i])) & 0xFF;
    $x ^= $x >> 4;
    $crc = (($crc << 8) ^ ($x << 12) ^ ($x << 5) ^ $x) & 0xFFFF;
  }
  return $crc;
}


function char2int($char) 
{
  $ret = 11;
  if ($char == '0') {$ret =  0;}
  if ($char == '1') {$ret =  1;}
  if ($char == '2') {$ret =  2;}
  if ($char == '3') {$ret =  3;}
  if ($char == '4') {$ret =  4;}
  if ($char == '5') {$ret =  5;}
  if ($char == '6') {$ret =  6;}
  if ($char == '7') {$ret =  7;}
  if ($char == '8') {$ret =  8;}
  if ($char == '9') {$ret =  9;}
  if ($char == '+') {$ret = 10;}
  return $ret;
}

function console_log($output, $with_script_tags = true) {
    $js_code = 'console.log(' . json_encode($output, JSON_HEX_TAG) . ');';
    if ($with_script_tags) {
        $js_code = '<script>' . $js_code . '</script>';
    }
    echo $js_code;
}

function validateDate($date)
{
    $d = DateTime::createFromFormat('Y-m-d H:i:s', $date);
    if($d && $d->format('Y-m-d H:i:s') == $date){
      return TRUE;
    }
    //error_log($date . " is invalid, " . $d->format('Y-m-d H:i:s'));
    return FALSE;
}

?>

