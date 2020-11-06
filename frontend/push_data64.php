<?php
    header("Access-Control-Allow-Origin: *");
    header("Access-Control-Allow-Methods: POST"); //alles wieder zu POST machen!!!!!!!!!!!!

    //Connect to database
    $servername = "localhost";
    $username   = "HeidiTracker";
    $password   = "12wanderfalke34!";
    $dbname     = "Heidi";
    $herdeID    = '';
    
    // Create connection
    $conn = new mysqli($servername, $username, $password, $dbname);
    // Check connection
    if ($conn->connect_errno) {
        echo "Database Connection failed: " . $conn->connect_error;
        die("Database Connection failed: " . $conn->connect_error);
    }
    $urlmarker = "X01";
    if(!empty($_POST[$urlmarker])){
    //if(isset($_POST['X01']){
      //php.ini: max_input_var = 1000 by default; post_max_size = 8M by default
      $stateOK    = TRUE;
      $i          = 1;
      $rounds     = 0; //prevent from running wild 
      while(($i > 0) && ($rounds < 100) && $stateOK){ 
        $rounds++;
        $urldata = $_POST[$urlmarker];
        //echo  $urldata . '<br>';
        $b64 = strtr($urldata, '-_', '+/');
        //echo $b64 . "<br>";
        $dec = base64_decode($b64, false);
        //echo bin2hex($dec) . "<br>";
        $data = unpack("C*", $dec);
        $pos = 1;
        $count = $data[$pos++];
        //echo "<br>" . $count . ", " . $pos . "<br>";
        if($count < 7 ) { continue; }     
        //uint_8
        $herde = $data[$pos++];
        if ($herde < 10) { $herde = '000' . $herde; } 
        elseif ($herde < 100) { $herde = '00' . $herde; } 
        elseif ($herde < 1000) { $herde = '0' . $herde; } 
        if ($herdeID === '') { $herdeID = $herde; }
        //echo $herde . ", " . $pos . "<br>";
        //uint_8
        $animal = $data[$pos++];
        if ($animal < 10) { $animal = '000' . $animal; } 
        elseif ($animal < 100) { $animal = '00' . $animal; } 
        elseif ($animal < 1000) { $animal = '0' . $animal; } 
        $id = $herde . '.' . $animal;
        //echo $animal . ", " . $pos . "<br>";
        //int_32 / 1000000
        $lat = (integer)($data[$pos++] + ($data[$pos++] << 8) + ($data[$pos++] << 16) + ($data[$pos++] << 24));
        $lat /= 1000000;
        //echo  $lat . ", " . $pos . "<br>";
        //int_32 / 1000000
        $lng = (integer)($data[$pos++] + ($data[$pos++] << 8) + ($data[$pos++] << 16) + ($data[$pos++] << 24));
        $lng /= 1000000;
        //echo  $lng . ", " . $pos . "<br>";
        //int_16 (needs special threadmet)
        if($data[$pos+1] > 127) 
        { $alt = (integer)($data[$pos++] + ($data[$pos++] << 8) + (0xFF << 16) + (0xFF << 24)); }
        else
        { $alt = (integer)($data[$pos++] + ($data[$pos++] << 8)); }
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
        //uint_16 / 1000
        $batt = (integer)($data[$pos++] + ($data[$pos++] << 8));
        $batt /= 1000;
        //echo  "batt: " . $batt . ", " . $pos . "<br>";
        $sql0 = "SELECT Count(*) FROM TrackerData WHERE TrackerID = '{$id}' AND TimeStamp = '{$date} {$time}';";
        $sql1 = "INSERT INTO TrackerData (TrackerID, Longitude, Latitude, Altitude, TimeStamp, Battery";
        $sql2 = "VALUES ('{$id}', '{$lng}', '{$lat}', '{$alt}', '{$date} {$time}', '{$batt}'";
        
        if ($count >= 8) {
          //uint_16 satellites
          $sql1 .= ', FreeValue1';
          $sat = (integer)($data[$pos++] + ($data[$pos++] << 8));
          //echo  "sat: " . $sat . ", " . $pos . "<br>";
          $sql2 .= ", '{$sat}'";
        } 

        if ($count >= 9) {
          //int_16 / 100 temperature
          $sql1 .= ', FreeValue2';
          if($data[$pos+1] > 127) 
          { $temp = (integer)($data[$pos++] + ($data[$pos++] << 8) + (0xFF << 16) + (0xFF << 24)); }
          else
          { $temp = (integer)($data[$pos++] + ($data[$pos++] << 8)); }
          $temp /= 100;
          //echo  "temp: " . $temp . ", " . $pos . "<br>";
          $sql2 .= ", '{$temp}'";
        } 

        if ($count >= 10) {
          //uint_16 error code
          $sql1 .= ', FreeValue3';
          $err = "0x" . strtoupper(dechex((integer)($data[$pos++] + ($data[$pos++] << 8))));
          //echo  "err: " . $err . ", " . $pos . "<br>";
          $sql2 .= ", '{$err}'";
        } 

        for ($x = 4; $x <= $count-7; $x++) {
          //uint_16 
          $sql1 .= ", FreeValue{$x}";
          $free = (integer)($data[$pos++] + ($data[$pos++] << 8));
          //echo  'FreeValue ' . $x . ': ' . $free . ", " . $pos . "<br>";
          $sql2 .= ", '{$free}'";
        } 

        //echo "{$sql1}) {$sql2})" . '<br><br>';
        if ($sqlres = $conn->query($sql0)){ 
          if ($row = $sqlres->fetch_row()){
            if ($row[0] == 0){
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
        if (i < 10){ $urlmarker = "X0{$i}"; } else { $urlmarker = "X{$i}"; }
        if (empty($_POST[$urlmarker])){ $i = 0; }
      }
      $i=1;
      //$fence_data = array():
      if ($stateOK === TRUE) {
        echo "OK";
        tellFence($conn, $herdeID);
      } else {
        echo "Error: " . $sql . "<br>" . $conn->error;
      }
    } else {
      echo "OK";
      if(!empty($_POST["ID"])){
        $herdeID = substr($_POST["ID"],0,4);
        tellFence($conn, $herdeID);
      } else { echo " - no valid Data"; }
    }
    $conn->close();

function tellFence($conn, $herdeID)
{
  $sql0 = "SELECT Longitude, Latitude FROM Fence WHERE HerdeID = '{$herdeID}' AND Active = 1;";
  if ($sqlres = $conn->query($sql0)){ 
    $fence_data = pack('C', (integer)$sqlres->num_rows);
    while ($row = $sqlres->fetch_row()){
      $fence_data .= pack('i', (integer)($row[0]*1000000)); 
      $fence_data .= pack('i', (integer)($row[1]*1000000)); 
    }
    $sqlres->close();
    $b64n = base64_encode($fence_data);
    $b64n = strtr($b64n, '+/', '-_');
    $b64n = preg_replace("/[^a-zA-Z0-9-_]/", "", $b64n);
    echo ';' . $b64n . ';' . dechex(crc16F($b64n));
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
?>

