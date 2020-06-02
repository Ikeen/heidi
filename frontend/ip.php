<?php
   header("Access-Control-Allow-Origin: *");
   header("Access-Control-Allow-Methods: GET");
   //$pageURL  = 'https://';
   //$pageURL .= gethostbyname($_SERVER["SERVER_NAME"]);
   //$pageURL .= ':1083/php/main_heidi.php';
   echo gethostbyname($_SERVER["SERVER_NAME"]);
?>
