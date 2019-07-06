<?php

    $pageURL  = 'https://';
    $pageURL .= gethostbyname($_SERVER["SERVER_NAME"]);
    $pageURL .= ':1083/php/main_heidi.php';
    echo $pageURL;
    header('Location: '.$pageURL);
?>
