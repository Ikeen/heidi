<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8"/>
<title>Heidi push settings</title>
</head>
<body bgcolor="#FFFFFF" text="#000000">
<font size=+1 face="monospace">
<?php
    header("Access-Control-Allow-Origin: *");
    header("Access-Control-Allow-Methods: POST, GET");
    //header("refresh:5");
    /* 
    $is_page_refreshed = (isset($_SERVER['HTTP_CACHE_CONTROL']) && $_SERVER['HTTP_CACHE_CONTROL'] == 'max-age=0');
    if($is_page_refreshed ) {
      echo 'This Page Is refreshed.';
    } else {
       echo 'This page is freshly visited. Not refreshed.';
    }
    */
    
    //Creates new record as per request
    //Connect to database
    $servername = "localhost";
    $username   = "HeidiTracker";
    $password   = "";
    $dbname     = "Heidi";
 
    // Create connection
    $conn = new mysqli($servername, $username, $password, $dbname);
    // Check connection
    if ($conn->connect_errno) {
        echo "Database Connection failed: " . $conn->connect_error;
        die("Database Connection failed: " . $conn->connect_error);
    }

    if (isset($_GET['ID'])) {$TrackerID = $_GET['ID'];} else {$TrackerID = "";}
    if(!empty($_GET['ID']) && !empty($_GET['Lo1']) && !empty($_GET['La1'])){
      $Table     = "Fence";
      $HerdeID   = substr($_GET["ID"],0,4);
      
      $sql1 = "DELETE FROM {$Table} WHERE HerdeID = '{$HerdeID}' AND Active = '0';";
      $sql2 = "UPDATE {$Table} SET Active = 0 WHERE HerdeID = '{$HerdeID}' AND Active = '1';";
      $sql3 = "INSERT INTO {$Table} (HerdeID, Longitude, Latitude, Active) ";
      $sql4 = "VALUES ('{$HerdeID}', ";
   
      if ($conn->query($sql1) === TRUE) {
        if ($conn->query($sql2) === TRUE) {
          for ($x = 1; $x <= 16; $x++) { //16 = max fence poles
            if(!empty($_GET["Lo{$x}"]) && !empty($_GET["La{$x}"])){
              $sql5 = $sql4;
              $sql5 .= $_GET["Lo{$x}"] . ", ";
              $sql5 .= $_GET["La{$x}"] . ", 1);";
              $sql = "{$sql3} {$sql5}";
              if ($conn->query($sql) === TRUE) { 
                //echo " -- success<br>"; 
              } else { 
                //echo " -- fail<br>"; 
                break; 
              }
            } else {
              break;
            }
          }
        } //else { echo "{$sql2} -- fail<br>"; }
      } //else { echo "{$sql1} -- fail<br>"; }
    }
    $conn->close();
?>
</font></body>
<script>
  var TrackerID = <?php echo json_encode($TrackerID); ?>;
  window.location.replace("heidimap.phtml?ID=" + TrackerID);
</script>
</html>


