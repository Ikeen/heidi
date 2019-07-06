<?php
    header("Access-Control-Allow-Origin: *");
    header("Access-Control-Allow-Methods: POST");

    //Creates new record as per request
    //Connect to database
    $servername = "localhost";
    $username   = "HeidiTracker";
    $password   = "12wanderfalke34!";
    $dbname     = "Heidi";
 
    // Create connection
    $conn = new mysqli($servername, $username, $password, $dbname);
    // Check connection
    if ($conn->connect_errno) {
        echo "Database Connection failed: " . $conn->connect_error;
        die("Database Connection failed: " . $conn->connect_error);
    }
 
    //Get current date and time
    //date_default_timezone_set('Asia/Kolkata');
    //$d = date("Y-m-d");
    //echo " Date:".$d."<BR>";
    //$t = date("H:i:s");
    //"?TrackerID=00001.00001&Longitude=-13.342804&Latitude=51.008919&Altitude=305&Date=2019%2F01%2F01&Time=10%3A30%3A00&Battery=3.32&FreeVal1=
 
    if(!empty($_POST['TrackerID']) && !empty($_POST['Longitude']) && !empty($_POST['Latitude']) && !empty($_POST['Altitude']) && !empty($_POST['Date']) && !empty($_POST['Time']) && !empty($_POST['Battery']))
    {
      $Table     = "TrackerData";
      $TrackerID = $_POST['TrackerID'];
      $Longitude = $_POST['Longitude'];
      $Latitude  = $_POST['Latitude'];
      $Altitude  = $_POST['Altitude'];
      $Date      = $_POST['Date'];
      $Time      = $_POST['Time'];
      $Battery   = $_POST['Battery'];
 
      $sql1 = "INSERT INTO {$Table} (TrackerID, Longitude, Latitude, Altitude, TimeStamp, Battery";
      $sql2 = "VALUES ('{$TrackerID}', '{$Longitude}', '{$Latitude}', '{$Altitude}', '{$Date} {$Time}', '{$Battery}'";

      for ($x = 1; $x <= 5; $x++) {
        if(!empty($_POST["FreeValue{$x}"])){
          $FreeVal = $_POST["FreeValue{$x}"];
          $sql1 .= ", FreeValue{$x}";
          $sql2 .= ", '{$FreeVal}'";
        }
      } 
 
      $sql = "{$sql1}) {$sql2})";

      if ($conn->query($sql) === TRUE) {
        if (empty($_POST['FromFormData'])){
          echo "OK<br>";
        } else {
          echo "<script>window.history.back();</script>";
        }
      } else {
	echo "Error: " . $sql . "<br>" . $conn->error;
      }

   } else if(!empty($_POST['ID1']) && !empty($_POST['Lo1']) && !empty($_POST['La1']) && !empty($_POST['Al1']) && !empty($_POST['Da1']) && !empty($_POST['Ti1']) && !empty($_POST['Ba1'])) {
      //short notation with multiple data sets
      //php.ini: max_input_var = 1000 by default; post_max_size = 8M by default
      $i = 1;
      $stateOK = TRUE;
      while(($i > 0) && $stateOK){
        $Table     = "TrackerData";
        $TrackerID = $_POST["ID{$i}"];
        $Longitude = $_POST["Lo{$i}"];
        $Latitude  = $_POST["La{$i}"];
        $Altitude  = $_POST["Al{$i}"];
        $Date      = $_POST["Da{$i}"];
        $Time      = $_POST["Ti{$i}"];
        $Battery   = $_POST["Ba{$i}"];
 
        $sql1 = "INSERT INTO {$Table} (TrackerID, Longitude, Latitude, Altitude, TimeStamp, Battery";
        $sql2 = "VALUES ('{$TrackerID}', '{$Longitude}', '{$Latitude}', '{$Altitude}', '{$Date} {$Time}', '{$Battery}'";

        for ($x = 1; $x <= 5; $x++) {
          if(!empty($_POST["F{$x}{$i}"])){
            $FreeVal = $_POST["F{$x}{$i}"];
            $sql1 .= ", FreeValue{$x}";
            $sql2 .= ", '{$FreeVal}'";
          }
        } 
 
        $sql = "{$sql1}) {$sql2})";
        $stateOK = $conn->query($sql);
        $i++;
        if(empty($_POST["ID{$i}"]) || empty($_POST["Lo{$i}"]) || empty($_POST["La{$i}"]) || empty($_POST["Al{$i}"]) || empty($_POST["Da{$i}"]) || empty($_POST["Ti{$i}"]) || empty($_POST["Ba{$i}"])){
          $i = 0;
        }
     }
      if ($stateOK === TRUE) {
        if (empty($_POST['FromFormData'])){
          echo "OK";
        } else {
          echo "<script>window.history.back();</script>";
        }
      } else {
	echo "Error: " . $sql . "<br>" . $conn->error;
      }
    }
    $conn->close();
?>
