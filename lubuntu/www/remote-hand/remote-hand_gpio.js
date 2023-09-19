/*
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2023.9.17
* remote-hand_pi_gpio.js  ver0.21 2022.9.17
*/
function blink(){
  if (!document.all){ return; }
  for (var i = 0; i < document.all.length; i++){
    obj = document.all(i);
    if (obj.className == "blink"){
      if (obj.style.visibility == "visible"){
        obj.style.visibility = "hidden";
      }
      else {
        obj.style.visibility = "visible";
      }
    }
  }
  setTimeout("blink()",1000);
}
var smapho_reload_tm = 10000;
var unsmapho_reload_tm = 60000;
var recognition = new webkitSpeechRecognition();
var recognition_state = "Stop"
var recognizing = false;
// It will initiate the voice recognition
function startWebVoiceRecognition(){
  if (!('webkitSpeechRecognition' in window)){
    alert("Sorry, your Browser does not support the Speech API");
  }
  else {
    if (recognizing === false){
      var voice_lang_val = $("#voice_lang").val();
      if (voice_lang_val == "en"){
        recognition.lang = "en";
      }
      else {
        recognition.lang = "ja";
      }
    }
   recognition.continuous = true;
//   recognition.continuous = false;

// Do not process intermediate results
   recognition.interimResults = false;
// recognition.interimResults = true;
    recognition.start();
    recognizing = true;
    recognition_state = "Please talk";
    initWebVoice(recognition_state);
    }
}

// In recognition of speech
recognition.onsoundstart = function(){
  if (recognizing === false){
    recognition.stop();
    recognition_state = "During stop processing";
  }
  else {
    recognition_state = "Recognition in";
  }
  $("#recognition_state").text(recognition_state);
}
// It does not recognize match
recognition.onnomatch = function(){
  recognition_state = "Please try again";
  $("#recognition_state").text(recognition_state);
}
// Any error
recognition.onerror= function(event){
  recognition.stop();
  recognizing = false;
  recognition_state = "Under suspension";
  $("#recognition_state").text(recognition_state);
}
// Recognition stop
recognition.onsoundend = function(){
  recognition.stop;
  recognizing = false;
  recognition_state = "Under suspension";
  $("#recognition_state").text(recognition_state);
}
recognition.onresult = function(event){
// Recognition end event
  var results = event.results;
  var results_voice = "";
  recognition_state = "It is processed";
  $("#recognition_state").text(recognition_state);
  if (event.results.length > 0){
    results_voice = results_voice + (results[0][0].transcript);
  }
  else return;
  var voice_lang_val = $("#voice_lang").val();
  if (recognizing === false){
    recognition_state = "It is processed";
    initWebVoice(recognition_state);
  }
  else {
    if (voice_lang_val != "en"){
      $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="' + results_voice + '" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="ja" SELECTED>Japanese<OPTION VALUE="en">English</SELECT>');
    }
    else {
      $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="' + results_voice + '" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="en" SELECTED>English<OPTION VALUE="ja">Japanese</SELECT>');
    }
    update_do("voice_sel",results_voice);
    stopWebVoiceRecognition();
/*    setTimeout(function(){
      recognition_state = "Please talk";
      $("#recognition_state").text(recognition_state);
    },5000);
*/
　 }
}
// End the speech recognition
function stopWebVoiceRecognition(){
  if (recognizing === true){
    recognition_state = "Stop";
    recognition.stop;
    recognizing = false;
　　　initWebVoice(recognition_state);
  }
  else {
    recognition_state = "Stop";
    $("#recognition_state").text(recognition_state);
  }
}
function initWebVoice(state){
  var voice_lang_val = $("#voice_lang").val();
  if (voice_lang_val != "en"){
    $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="ja" SELECTED>Japanese<OPTION VALUE="en">English</SELECT>');
    } else {
    $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="en" SELECTED>English<OPTION VALUE="ja">Japanese</SELECT>');
  }
  $("#recognition_state").text(state);
}
function CheckBrowser(){
  var userAgent = window.navigator.userAgent.toLowerCase();
  if(userAgent.indexOf('edg') != -1)     { return "Edge";   }
  if(userAgent.indexOf('chrome') != -1)   { return "Chrome";  }
  if(userAgent.indexOf('safari') != -1)    { return "Safari";  }
  if(userAgent.indexOf('iphone') != -1)    { return "Safari";  }
  if(userAgent.indexOf('ipad') != -1)    { return "Safari";  }
  if(userAgent.indexOf('firefox') != -1)  { return "FireFox"; }
  if(userAgent.indexOf('opera') != -1)    { return "Opera";   }
  return "Unknown";
}
function speak_exec(voice,lang){
  var Browser = CheckBrowser();
  var lang_temp;
  if(Browser == "Chrome" | Browser == "FireFox" | Browser == "Edge" | Browser == "Safari"){
    var utterance = new SpeechSynthesisUtterance();
    if (lang == "en") lang_temp = "en-US";
    if (lang == "ja") lang_temp = "ja";
    utterance.text = voice;
    utterance.lang = lang_temp;
    utterance.pitch = 1.2;
    utterance.rate = 0.8;
    speechSynthesis.speak(utterance);
  }
}
function speak_main(voice,lang){
  speak_exec(voice,lang);
  $(function(){
    setTimeout(function(){
      if (lang != "en"){
        $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="ja" SELECTED>Japanese<OPTION VALUE="en">English</SELECT>');
        } else {
        $("#voice_sel").html('Voice control<input id="voice_val" type="text" style="width:120px;" NAME="voice_val" VALUE="" onkeydown="if(event.keyCode == 13 || event.keyCode == 9) update_do(\'voice_sel\')" placeholder="Command" autofocus /><SELECT NAME="voice_lang" id="voice_lang"><OPTION VALUE="en" SELECTED>English<OPTION VALUE="ja">Japanese</SELECT>');
      }
    },5000);
  });
}
function google_speak(voice_t,voice_l){
  if (voice_l == "en"){
    voice_t = "OK to run the" + voice_t;
  }
  else {
    voice_t = "" + voice_t + "を実行します。";
  }
  speak_main(voice_t,voice_l);
}
function google_speak_none(voice_t,voice_l){
  if (voice_l == "en"){
//    voice_t = voice_t + " do not understand";
      voice_t = "Because" + voice_t + "can not understand, I searched it with google";
  }
  else {
    var search_val = voice_t;
    voice_t = voice_t + "、をウェブで検索しました";
  }
    var child_url = "https://www.google.com/search?q=" + search_val;
      var google_search = window.open(child_url,"width=640,height=480,resizable=yes,scrollbars=no");
      setTimeout(function () {
        google_search .close();
      },15000);
    speak_main(voice_t,voice_l);
}

// IR data registration processing of IRKit
function irkit_reg(ir_num,ir_id){
  var ir_val = $(ir_id).val();
  if (ir_val == "none") return;
  var ir_reg = ir_num;
  var ir_req = ir_id;
  switch (ir_reg){
    case 'irdata_0':
      req_ir = "0";
      break;
    case 'irdata_1':
      req_ir = "1";
      break;
    case 'irdata_2':
      req_ir = "2";
      break;
    case 'irdata_3':
      req_ir = "3";
      break;
    case 'irdata_4':
      req_ir = "4";
      break;
    case 'irdata_5':
      req_ir = "5";
      break;
  }
  $.ajax({
    type: "get",
    url: "irkit_reg.cgi",
    timeout : 3000,
    dataType: "text",
    async: true,
    data: 'ir_num=' + req_ir,
    success: function(){
      $("#irkit_ip").text("Server-Success!");
    },
    error: function(){
      $("#irkit_ip").text("Server-Timeout!");
    }
  });
}

// IR data output processing of IRKit
function irkit_post(do_ch,do_time){
  var ir_timer = do_time;
  var ir_num;
  switch (do_ch){
    case '8':
      ir_num = 0;
      break;
    case '9':
      ir_num = 1;
      break;
    case '10':
      ir_num = 2;
      break;
    case '11':
      ir_num = 3;
      break;
    case '12':
      ir_num = 4;
      break;
    case '13':
      ir_num = 5;
      break;
  }
  $.ajax({
    type: "get",
    url: "irkit_post.cgi",
    timeout : 3000,
    dataType: "text",
    async: true,
    data: 'ir_num=' + ir_num + '&ir_timer=' + ir_timer,
    success: function(){
      $("#irkit_ip").text("Server-Success!");
    },
    error: function(){
      $("#irkit_ip").text("Server-Timeout!");
    }
  });
}
// IR data display of IRKit
function disp_irdata(irdata,val){
  var color_bg;
  var color_font;
  if (val == "Ready"){
  // Background Green
    color_bg = "#008000";
    color_font = "#F0FFFF";
  }
  if (val == "none"){
  // Background gray
    color_bg = "#A9A9A9";
    color_font = "#000000";
  }
  $(irdata).html('<INPUT TYPE="text" size="3" name="' + irdata + '" readonly style="color:' + color_font + ';background-color:' + color_bg + '"width:50px;text-align:Left;" VALUE="' + val + '">&nbsp;');
}
// IP address search of IRKit
function irkit_search(){
  var irkit_ip = document.menu5.irkit_ip.value;
/*  if (ipaddr_ck(irkit_ip) == -1){
    irkit_ip = "none";
  }
*/
  $.ajax({
    url: "irkit_search.cgi",
    dataType: "text",
    type: "GET",
    cache: true,
    timeout : 1000,
    async: true,
    data: "ip=" + irkit_ip,
    success: function(){
      $("#irkit_ip").text("Server-Success!");
    },
    error: function(){
      $("#irkit_ip").text("Server-Timeout!");
   }
  });
}
// Log display of digital input
function disp_di_log(url_di_log){
  var disp_url = "tmp/." + url_di_log + ".log";
  var server_date = $("#server_time").val();
  $.ajax({
      url: disp_url,
      dataType: "text",
      type: "GET",
      cache: true,
      async: true,
      timeout : 1000,
      success: function(data){
        $(function(){
          var di_log = window.open("","di_log","width=300,height=500,resizable=yes,scrollbars=yes");
          di_log.document.write("<title>" + url_di_log + " Update Log</title>");
          di_log.document.write("<pre>");
          di_log.document.write(url_di_log + " Update Log " + server_date + "<br>");
          di_log.document.write(data);
          di_log.document.write("</pre>");
          return true;
        });
      },
      error: function(status){
        alert("It failed to log acquisition");
        return false;
      }
   });
}
// Processing of live photo
function start_photo(dev){
  var ip = $("#live_server").val();
  var live_close_timer = 60000;
  var live_timer = 0;
  var video_dev = dev;
  var liveimg_status = "none";
  var liveimg = "remote-hand.jpg";
  var json_interval_timer = 3000;
  var interval_cout = 0;
  var interval_max = 10;
  var live_url = "/remote-hand/tmp/remote-hand.jpg";
  var wait_url ="/remote-hand/please_wait.html";
  var wait_photo = null;
  function wait_img(){
    if (wait_photo === null)
    {
      var child_url = "http://" + ip + wait_url;
      wait_photo = window.open(child_url,dev,"width=320,height=240,resizable=yes,scrollbars=no");
    }
    $.ajax({
      url: ".di_read_data.json",
      dataType: "json",
      type: "get",
      cache: true,
      async: true,
      timeout : 3000,
      success: function(di2json, status){
        $(function(){
          if (status == "success"){
            liveimg_status = di2json.liveimg;
            var id = setTimeout(function(){
              if (liveimg_status == liveimg){
                clearTimeout(id);
                wait_photo.close();
                child_url = "http://" + ip + live_url;
                var live_photo = window.open(child_url,dev,"width=320,height=240,resizable=yes,scrollbars=no");
                var id = setTimeout(function(){
                  live_photo.close();
                },live_close_timer);
                return true;
              } else {
                if (interval_cout++ < interval_max){
                  wait_img();
                }
                else {
                  wait_photo.close();
                  clearTimeout(id);
                  return false;
                }
              }
            },json_interval_timer);
          }
        });
      },
      error: function(){
        $("#disp_menu5").text("Server-Timeout Live Photo!");
        return false;
      }
    });
  }
  $.ajax({
    type: "get",
    url: "pepoliveserver.cgi",
    timeout : 3000,
    dataType: "text",
    async: true,
    data: 'dev=' + video_dev + '&live_timer=' + live_timer,
    success: function(){
      $("#disp_menu5").text("Server-Success!");
    },
    error: function(){
      $("#disp_menu5").text("Server-Timeout Live Photo!");
      return false;
    }
  });
  wait_img();
}
// Processing of live video
function start_video(dev){
  var ip = $("#live_server").val();
  var live_timer;
  var live_close_timer;
  var live_open_timer;
  var video_dev = dev;
  var live_url = '/remote-hand/tmp/remote-hand.webm'
  var wait_url ="/remote-hand/please_wait.html";
  var liveimg_status = "none";
  var liveimg = "remote-hand.webm";
  var json_interval_timer = 3000;
  var interval_cout = 0;
  var interval_max = 10;
  if (video_dev == "video0"){
    live_timer = $('#live_timer0').val();
  }
  else if (video_dev == "video1"){
    live_timer = $('#live_timer1').val();
  }
  else if (video_dev == "video2"){
    live_timer = $('#live_timer2').val();
  }
  else if (video_dev == "vchiq"){
    live_timer = $('#live_timer3').val();
  }
  if (live_timer === undefined){
    live_timer = 10;
  }
  live_timer = live_timer * 1 + 5;
  if (video_dev == "vchiq"){
    live_close_timer = live_timer * 1000 * 4.0;
  }
  else {
    live_close_timer = live_timer * 1000 * 4.0;
  }
  var wait_photo = null;
  function wait_img(){
    if (wait_photo === null)
    {
      var child_url = "http://" + ip + wait_url;
      wait_photo = window.open(child_url,dev,"width=320,height=240,resizable=yes,scrollbars=no");
    }
    $.ajax({
      url: ".di_read_data.json",
      dataType: "json",
      type: "get",
      cache: true,
      async: true,
      timeout : 3000,
      success: function(di2json, status){
        $(function(){
          if (status == "success"){
            liveimg_status = di2json.livevideo;
            var id = setTimeout(function(){
              if (liveimg_status == liveimg){
                clearTimeout(id);
                wait_photo.close();
                child_url = "http://" + ip + live_url;
                var live_photo = window.open(child_url,dev,"width=320,height=240,resizable=yes,scrollbars=no");
                var id = setTimeout(function(){
                  live_photo.close();
                },live_close_timer);
                return true;
              } else {
                if (interval_cout++ < interval_max){
                  wait_img();
                }
                else {
                  wait_photo.close();
                  clearTimeout(id);
                  return false;
                }
              }
            },json_interval_timer);
          }
        });
      },
      error: function(){
        $("#disp_menu5").text("Server-Timeout Live Photo!");
        return false;
      }
    });
  }
  $.ajax({
    type: "get",
    url: "pepoliveserver.cgi",
    timeout : 3000,
    dataType: "text",
    async: true,
    data: 'dev=' + video_dev + '&live_timer=' + live_timer,
    success: function(){
      $("#disp_menu5").text("Server-Success!");
    },
    error: function(){
      $("#disp_menu5").text("Server-Timeout Live Photo!");
      return false;
    }
  });
  wait_img();
}
// Module Camera streaming start,stop
function streaming_start_stop(dev,start_stop){
  var server_ip = $("#live_server").val();
  $.ajax({
    type: "get",
    url: "streming_start_stop.cgi",
    timeout : 3000,
    dataType: "text",
    async: true,
    data: 'dev=' + dev + '&start_stop=' + start_stop,
    success: function(){
      $("#disp_menu5").text("Server-Success!");
      $(function(){
        if (start_stop == "start"){
          setTimeout(function(){
            alert("Please be Start the vlc player , http://" + server_ip + ":8554 please visit");
          },5000);
        }
        else {
          setTimeout(function(){
            alert("It has stopped streaming Please be Stop the vlc player");
          },2000);
        }
        return true;
      });
    },
    error: function(){
      $("#disp_menu5").text("Server-Timeout " + start_stop + "streming");
      return false;
    }
  });
}
function send_do(do_ch,do_val,do_time){
  if (do_time === undefined){do_time = ""}
  if (do_ch >= 8 && do_ch <= 13){
    irkit_post(do_ch,do_time);
  }
  else {
    $.ajax({
      type: "GET",
      url: "do_ajax.cgi",
      timeout : 3000,
      dataType: "text",
      data: 'ch=' + do_ch + '&val=' + do_val + '&time=' + do_time,
      success: function(){
        $("#disp_menu5").text("Digtal Out Success!");
      },
      error: function(do_sel){
        $("#disp_menu5").text("Server-Timeout!");
       }
    });
  }
}
// Digital output process for smart phone
function s_phone_update_do(tdo_id){
  var tdo_ch = "none";
  var high = "1";
  var low = "0";
  var tdo_val = low;
  var do_id_val = "none";
  var tdo_time = "";
  $.ajax({
     url: ".di_read_data.json",
     dataType: "json",
     type: "get",
     cache: true,
     async: true,
     timeout : 3000,
     success: function(di2json, status){
       if (status == "success"){
         switch (tdo_id){
           case 'dosel_0':
             do_id_val = di2json.do0;
             tdo_ch = "0";
             break;
           case 'dosel_1':
             do_id_val = di2json.do1;
             tdo_ch = "1";
             break;
           case 'dosel_2':
             do_id_val = di2json.do2;
             tdo_ch = "2";
             break;
           case 'dosel_3':
             do_id_val = di2json.do3;
             tdo_ch = "3";
             break;
           case 'dosel_4':
             do_id_val = di2json.do4;
             tdo_ch = "4";
             break;
           case 'dosel_5':
             do_id_val = di2json.do5;
             tdo_ch = "5";
             break;
           case 'dosel_6':
             do_id_val = di2json.do6;
             tdo_ch = "6";
             break;
           case 'dosel_7':
             do_id_val = di2json.do7;
             tdo_ch = "7";
             break;
           case 'irkitdo_0':
             do_id_val = di2json.irdata_0;
             tdo_ch = "8";
             if (do_id_val == "none") return;
             break;
           case 'irkitdo_1':
             do_id_val = di2json.irdata_1;
             tdo_ch = "9";
             if (do_id_val == "none") return;
             break;
           case 'irkitdo_2':
             do_id_val = di2json.irdata_2;
             tdo_ch = "10";
             if (do_id_val == "none") return;
             break;
           case 'irkitdo_3':
             do_id_val = di2json.irdata_3;
             tdo_ch = "11";
             if (do_id_val == "none") return;
             break;
           case 'irkitdo_4':
             do_id_val = di2json.irdata_4;
             tdo_ch = "12";
             if (do_id_val == "none") return;
             break;
           case 'irkitdo_5':
             do_id_val = di2json.irdata_5;
             tdo_ch = "13";
             if (do_id_val == "none") return;
             break;
           case 'tocosdo_1':
             do_id_val = di2json.to1;
             tdo_ch = "14";
             break;
           case 'tocosdo_2':
             do_id_val = di2json.to2;
             tdo_ch = "15";
             break;
           case 'tocosdo_3':
             do_id_val = di2json.to3;
             tdo_ch = "16";
             break;
          }
        }
        if (do_id_val == "high"){
          tdo_val = "0";
        }
        if (do_id_val == "low" || do_id_val == "none"){
           tdo_val = "1";
        }
        send_do(tdo_ch,tdo_val,tdo_time);
      },
      error: function(do_id_val){
        $("#disp_menu5").text("Server-Timeout!");
        return;
       }
    });
}
function sendmail_pic(pic_val,mail_address){
  var mail_cgi = "none";
  if (pic_val == "picture") mail_cgi = "pepogmail4jpg.cgi";
  if (pic_val == "video") mail_cgi = "pepogmail4pic.cgi";
  mail_address = "mail_address=" + mail_address;
  $.ajax({
    type: "GET",
    url: mail_cgi,
    timeout : 3000,
    dataType: "text",
    data: mail_address,
    success: function(){
      $("#disp_menu5").text("Mail send Success!");
    },
    error: function(do_sel){
      $("#disp_menu5").text("Server-Timeout!");
    }
  });
}
function voice_do(do_sel,results_voice){
  var voice_src = results_voice;
  $.ajax({
    url: ".di_read_data.json",
    dataType: "json",
    type: "get",
    cache: true,
    async: true,
    timeout : 5000,
    success: function(di2json){
      var str = "";
      var tdo_val = "";
      var tdo_val_tmp = "";
      var tdo_id = "";
      var tdo_ch = "";
      var tdo_time = "";
      var tvoice_ck = "";
      var voice_lang = $('#voice_lang').val();
      var voice_str = voice_src.replace(/\s+/g, "");
      var array_voice_alias = new Array(80);
      var voice_tmp = voice_str;
      voice_str = unescape(escape(voice_str).replace(/^(%u3000|%20|%09)+|(%u3000|%20|%09)+$/g, ""));
      if (voice_lang == "en"){
            voice_str = voice_src.toLowerCase();
      }
      tdo_time = "";
      if (voice_str == "写真"　|| voice_str == "写真を撮って"　|| voice_str == "写真とって"　|| voice_str == "写真見せて"　||
      　　voice_str == "写真を見せて"　|| voice_str == "写真みせて"){
        tdo_val = "start_picture";
      }            　
      if (voice_str == "動画" || voice_str == "動画を撮って" || voice_str == "動画撮って" ||  voice_str == "動画見せて" ||
        voice_str == "動画を見せて"　|| voice_str == "動画をみせて"){
        tdo_val = "start_video";
      }
      if (voice_str == "写真を送って" || voice_str == "写真送信して" || voice_str == "写真おくって"){
        tdo_val = "picture";
      }
      if (voice_str == "動画を送って"　||　voice_str == "動画送って" || voice_str == "動画おくって"){
        tdo_val = "video";
      }
      if (voice_str == "takepicture" || voice_str == "picture"){
        tdo_val = "start_picture";
      }
      if (voice_str == "video" || voice_str == "avideo" ||
        voice_str == "takevideo" || voice_str == "takeavideo"){
        tdo_val = "start_video";
      }
// "send me photo","send me picture","take photo","take picture","picture"
      if (voice_str == "sendmephoto" || voice_str == "sendmepicture" || voice_str == "sendapicture" ||
        voice_str == "photo" || voice_str == "sepicture"){
        tdo_val = "picture";
      }
// "send mep video","send me video","video","take video","video";
      if (voice_str == "sendmevideo" || voice_str == "sendmeavideo" || voice_str == "sendavideo"){
        tdo_val = "video";
      }
      if (tdo_val == "picture" || tdo_val =="video"){
        var mail_address;
        mail_address = $('#di2json.voice_mail').val();
        if (mail_ck(mail_address) != -1){
          sendmail_pic(tdo_val,mail_address);
        }
      else {
        if (voice_lang == "en"){
          var voice_tmp = "It is incomplete e-mail address"
        } else {
          var voice_tmp = "メールアドレスが不完全です"
    　   }
      }
      speak_main(voice_tmp,voice_lang);
      return;
      }
      if (tdo_val == "start_picture"){
        if (voice_lang == "en"){
          voice_tmp = "Ok I will show you a picture";
        } else {
          voice_tmp = "写真表示";
        }
        google_speak(voice_tmp,voice_lang);
        start_picture("video0");
        return;
      }
      if (tdo_val == "start_video"){
        if (voice_lang == "en"){
          voice_tmp = "Ok I will show you a video";
        } else {
          voice_tmp = "動画表示";
        }
        google_speak(voice_tmp,voice_lang);
        start_video("video0");
        return;
      }
      array_voice_alias[0] = di2json.alias_do0;
      array_voice_alias[1] = di2json.alias_do1;
      array_voice_alias[2] = di2json.alias_do2;
      array_voice_alias[3] = di2json.alias_do3;
      array_voice_alias[4] = di2json.alias_do4;
      array_voice_alias[5] = di2json.alias_do5;
      array_voice_alias[6] = di2json.alias_do6;
      array_voice_alias[7] = di2json.alias_do7;
      array_voice_alias[8] = di2json.alias_do8;
      array_voice_alias[9] = di2json.alias_do9;
      array_voice_alias[10] = di2json.alias_do10;
      array_voice_alias[11] = di2json.alias_do11;
      array_voice_alias[12] = di2json.alias_do12;
      array_voice_alias[13] = di2json.alias_do13;
      array_voice_alias[14] = di2json.alias_do14;
      array_voice_alias[15] = di2json.alias_do15;
      array_voice_alias[16] = di2json.alias_do16;
      array_voice_alias[17] = di2json.alias_do17;
      array_voice_alias[18] = di2json.alias_do18;
      array_voice_alias[19] = di2json.alias_do19;
      array_voice_alias[20] = di2json.alias_do20;
      array_voice_alias[21] = di2json.alias_do21;
      array_voice_alias[22] = di2json.alias_do22;
      array_voice_alias[23] = di2json.alias_do23;
      array_voice_alias[24] = di2json.alias_do24;
      array_voice_alias[25] = di2json.alias_do25;
      array_voice_alias[26] = di2json.alias_do26;
      array_voice_alias[27] = di2json.alias_do27;
      array_voice_alias[28] = di2json.alias_do28;
      array_voice_alias[29] = di2json.alias_do29;
      array_voice_alias[30] = di2json.alias_do30;
      array_voice_alias[31] = di2json.alias_do31;
      array_voice_alias[32] = di2json.alias_do32;
      array_voice_alias[33] = di2json.alias_do33;
      array_voice_alias[34] = di2json.alias_do34;
      array_voice_alias[35] = di2json.alias_do35;
      array_voice_alias[36] = di2json.alias_do36;
      array_voice_alias[37] = di2json.alias_do37;
      array_voice_alias[38] = di2json.alias_do38;
      array_voice_alias[39] = di2json.alias_do39;
      array_voice_alias[40] = di2json.alias_do40;
      array_voice_alias[41] = di2json.alias_do41;
      array_voice_alias[42] = di2json.alias_do42;
      array_voice_alias[43] = di2json.alias_do43;
      array_voice_alias[44] = di2json.alias_do44;
      array_voice_alias[45] = di2json.alias_do45;
      array_voice_alias[46] = di2json.alias_do46;
      array_voice_alias[47] = di2json.alias_do47;
      array_voice_alias[48] = di2json.alias_do48;
      array_voice_alias[49] = di2json.alias_do49;
      array_voice_alias[50] = di2json.alias_do50;
      array_voice_alias[51] = di2json.alias_do51;
      array_voice_alias[52] = di2json.alias_do52;
      array_voice_alias[53] = di2json.alias_do53;
      array_voice_alias[54] = di2json.alias_do54;
      array_voice_alias[55] = di2json.alias_di0;
      array_voice_alias[56] = di2json.alias_di1;
      array_voice_alias[57] = di2json.alias_di2;
      array_voice_alias[58] = di2json.alias_di3;
      array_voice_alias[59] = di2json.alias_di4;
      array_voice_alias[60] = di2json.alias_di5;
      array_voice_alias[61] = di2json.alias_di6;
      array_voice_alias[62] = di2json.alias_di7;
      array_voice_alias[63] = di2json.alias_di8;
      array_voice_alias[64] = di2json.alias_di9;
      array_voice_alias[65] = di2json.alias_di10;
      array_voice_alias[66] = di2json.alias_di11;
      array_voice_alias[67] = di2json.alias_di12;
      array_voice_alias[68] = di2json.alias_di13;
      array_voice_alias[69] = di2json.alias_di14;
      array_voice_alias[70] = di2json.alias_di15;
      array_voice_alias[71] = di2json.alias_di16;
      array_voice_alias[72] = di2json.alias_di17;
      array_voice_alias[73] = di2json.alias_di18;
      array_voice_alias[74] = di2json.alias_di19;
      array_voice_alias[75] = di2json.alias_di20;
      array_voice_alias[76] = di2json.alias_di21;
      array_voice_alias[77] = di2json.alias_di22;
      array_voice_alias[78] = di2json.alias_di23;
      array_voice_alias[79] = "ありがとう";
      array_voice_alias[80] = "有難う";
      array_voice_alias[81] = "えらいね";
      array_voice_alias[82] = "えらいねぇ";
      array_voice_alias[83] = "偉いね";
      tdo_val = "none";
      for (var i=0; i <= 83; i++){
        str = array_voice_alias[i];
        if (str == "none") continue;
        if (voice_lang == "en"){
          str = str.toLowerCase();
        }
        tvoice_ck = str　+ "オン";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "おん";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "ON";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "音";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "をつけて";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "つけて";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "回して";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "まわして";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "を回して";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "をまわして";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "点灯";
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = str　+ "オフ";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "おふ";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "OFF";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "をけして";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "を消して";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
         }
        tvoice_ck = str　+ "けして";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "消して";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "を決して";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "決して";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "を消灯";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "消灯";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "とめて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "止めて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "をとめて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "を止めて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "やめて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "をやめて";
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = str　+ "おしえて";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "をおしえて";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "教えて";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "を教えて";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "は";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "表示して";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "を表示して";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "表示";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = str　+ "を表示";
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Tell me the"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Tell me"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Show"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Show the"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Display"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck =  "Display the"　+ str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "input_disp";
          break;
        }
        tvoice_ck = "You can turn on the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = "You can turn off the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = "run the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = "stop the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = "run" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = "stop" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = "turn on the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = "power on the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "1";
          break;
        }
        tvoice_ck = "turn off the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        tvoice_ck = "power off the" + str;
        tvoice_ck = tvoice_ck.replace(/\s+/g, "");
        if (tvoice_ck == voice_str){
          tdo_val = "0";
          break;
        }
        if (tdo_val == "none" && voice_str == str){
          if (i >= 0 && i <= 3){ // DIO
            tdo_val = "half";
          }
          else if (i >= 21 && i <= 35){ // DIO
            tdo_val = "half";
          }
          else if (i >= 8 && i <= 13){ // IRKit
            tdo_val = "1";
          }
          else if (i >= 36 && i <= 47){ // IRKit
            tdo_val = "1";
          }
          else if (i >= 14 && i <= 16){ // Tocos
            tdo_val = "half";
          }
          else if (i >= 48 && i <= 54){ // Tocos
            tdo_val = "half";
          }
          else if (i >= 55 && i <= 83){ // input disp
            tdo_val = "input_disp";
          }
          break;
        }
      }
      if (i == 0 || i == 21 || i == 22){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do0;
        } else {
          tdo_ch = "0";
          tdo_id = "do0";
          tdo_time = $('#don_time_0').val();
        }
      }
      if (i == 1 || i == 23 || i == 24){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do1;
        } else {
          tdo_ch = "1";
          tdo_id = "do1";
          tdo_time = $('#don_time_1').val();
        }
      }
      if (i == 2 || i == 25 || i == 26){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do2;
        } else {
          tdo_ch = "2";
          tdo_id = "do2";
          tdo_time = $('#don_time_2').val();
        }
      }
      if (i == 3 || i == 27 || i == 28){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do3;
        } else {
          tdo_ch = "3";
          tdo_id = "do3";
          tdo_time = $('#don_time_3').val();
        }
      }
      if (i == 4 || i == 29 || i == 30){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do4;
        } else {
          tdo_ch = "4";
          tdo_id = "do4";
          tdo_time = $('#don_time_4').val();
        }
      }
      if (i == 5 || i == 31 || i == 32){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do5;
        } else {
          tdo_ch = "5";
          tdo_id = "do5";
          tdo_time = $('#don_time_5').val();
        }
      }
      if (i == 6 || i == 33 || i == 34){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do6;
        } else {
          tdo_ch = "6";
          tdo_id = "do6";
          tdo_time = $('#don_time_6').val();
        }
      }
      if (i == 7 || i == 35 || i == 36){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.do7;
        } else {
          tdo_ch = "7";
          tdo_id = "do7";
          tdo_time = $('#don_time_7').val();
        }
      }
      if (i == 8 || i == 37 || i == 38){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_0;
        } else {
          tdo_ch = "8";
          tdo_id = "do8";
          tdo_time = $('#don_time_8').val();
        }
     }
      if (i == 9 || i == 39 || i == 40){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_1;
        } else {
          tdo_ch = "9";
          tdo_id = "do9";
          tdo_time = $('#don_time_9').val();
        }
      }
      if (i == 10 || i == 41 || i == 42){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_2;
        } else {
          tdo_ch = "10";
          tdo_id = "do10";
          tdo_time = $('#don_time_10').val();
        }
      }
       if (i == 11 || i == 43 || i == 44){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_3;
        } else {
          tdo_ch = "11";
          tdo_id = "do11";
          tdo_time = $('#don_time_11').val();
        }
      }
      if (i == 12 || i == 45 || i == 46){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_4;
        } else {
          tdo_ch = "12";
          tdo_id = "do12";
          tdo_time = $('#don_time_12').val();
        }
      }
      if (i == 13 || i == 47 || i == 48){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.irdata_5;
        } else {
          tdo_ch = "13";
          tdo_id = "do13";
          tdo_time = $('#don_time_13').val();
        }
      }
      if (i == 14 || i == 49 || i == 50){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.to1;
        } else {
          tdo_ch = "14";
          tdo_id = "to1";
          tdo_time = $('#don_time_14').val();
        }
      }
      if (i == 15 || i == 51 || i == 52){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.to2;
        } else {
          tdo_ch = "15";
          tdo_id = "to2";
          tdo_time = $('#don_time_15').val();
        }
      }
      if (i == 16 || i == 53 || i == 54){
        if (tdo_val == "input_disp"){
          tdo_ch = array_voice_alias[i];
          tdo_id = di2json.to3;
        } else {
          tdo_ch = "16";
          tdo_id = "to3";
          tdo_time = $('#don_time_16').val();
        }
      }
      if (i == 55){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio0";
        } else {
          tdo_ch = array_voice_alias[55];
          tdo_id = di2json.di0;  
        }
      }
      if (i == 56){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio1";
        } else {
          tdo_ch = array_voice_alias[56];
          tdo_id = di2json.di1;
        }
      }
      if (i == 57){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio2";
        } else {
          tdo_ch = array_voice_alias[57];
          tdo_id = di2json.di2;
        }
      }
      if (i == 58){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio3";
        } else {
          tdo_ch = array_voice_alias[58];
          tdo_id = di2json.di3;
        }
      }
      if (i == 59){
        tdo_ch = array_voice_alias[59];
        tdo_id = di2json.di4;
      }
      if (i == 60){
          tdo_ch = array_voice_alias[60];
          tdo_id = di2json.di5;
      }
      if (i == 61){
          tdo_ch = array_voice_alias[61];
          tdo_id = di2json.di6;
      }
      if (i == 62){
        tdo_ch = array_voice_alias[62];
        tdo_id = di2json.di7;
      }
      if (i == 63){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio8";
        } else {
          tdo_ch = array_voice_alias[63];
          tdo_id = di2json.ti1;
        }
      }
      if (i == 64){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio9";
        } else {
          tdo_ch = array_voice_alias[64];
          tdo_id = di2json.ti2;
        }
      }
      if (i == 65){
        if (tdo_val != "input_disp"){
          tdo_ch = "dio10";
        } else {
          tdo_ch = array_voice_alias[65];
          tdo_id = di2json.ti3;
        }
      }
      if (i == 66){
        tdo_ch = array_voice_alias[66];
        tdo_id = "none";
      }
      if (i == 67){
        tdo_ch = array_voice_alias[67];
        tdo_id = di2json.ai2di1;
      }
      if (i == 68){
        tdo_ch = array_voice_alias[68];
        tdo_id = di2json.ai2di2;
      }
      if (i == 69){
        tdo_ch = array_voice_alias[69];
        tdo_id = di2json.ai2di3;
      }
      if (i == 70){
        tdo_ch = array_voice_alias[70];
        tdo_id = di2json.ai2di4;
      }
      if (i == 71){
        tdo_ch = array_voice_alias[71];
        tdo_id = di2json.cpu_temp;
      }
      if (i == 72){
        tdo_ch = array_voice_alias[72];
        tdo_id = di2json.gpio_i2c.temp;
      }
      if (i == 73){
        tdo_ch = array_voice_alias[73];
        tdo_id = di2json.gpio_i2c.hum;
      }
      if (i == 74){
        tdo_ch = array_voice_alias[74];
        tdo_id = di2json.i2ctemp.temp;
      }
      if (i == 75){
        tdo_ch = array_voice_alias[75];
        tdo_id = di2json.i2ctemp.hum;
      }
      if (i == 76){
        tdo_ch = array_voice_alias[76];
        tdo_id = di2json.gpio_i2c.pres;
        if (tdo_id != "none"){
          tdo_id = tdo_id.replace(/[^0-9]/g,"");
          tdo_id = tdo_id + "ヘクトパスカル";
        }
      }
      if (i == 77){
        tdo_ch = array_voice_alias[77];
        tdo_id = di2json.gpio_i2c.gas;
      }
      if (i == 78){
        tdo_ch = array_voice_alias[78];
        var tdo_temp = di2json.gpio_i2c.temp;
        var tdo_hum = di2json.gpio_i2c.hum;
        var tdo_pres = di2json.gpio_i2c.pres;
        var tdo_iaq = di2json.gpio_i2c.iaq;
        if (tdo_iaq != "none"){
          var iaq_val = tdo_iaq;
          var iaq_color = "";
          if ( iaq_val <= 50) {
            iaq_color = "良い";
          } else if ( iaq_val <= 100) {
            iaq_color = "普通";
          } else if ( iaq_val <= 150) {
            iaq_color = "少し悪い";
          } else if ( iaq_val <= 200) {
             iaq_color = "かなり悪い";
          } else if ( iaq_val <= 300) {
            iaq_color = "かなり悪い";
          } else if ( iaq_val > 300) {
            iaq_color = "大変悪い";
          }
          tdo_id = tdo_iaq + "で" + iaq_color + "です";
        }
         voice_tmp = tdo_ch + ",温度," + tdo_temp + ",湿度," + tdo_hum + ",気圧," +　tdo_pres + ",空気質," + tdo_id;
　　      speak_main(voice_tmp,voice_lang);
        return;
      }
      if (i == 79 || i == 80){
         voice_tmp = "どう致しまして";
　　      speak_main(voice_tmp,voice_lang);
        return;
      }
      if (i == 81 || i == 82|| i == 83){
         voice_tmp = "お役に立てて光栄です";
　　      speak_main(voice_tmp,voice_lang);
        return;
      }
      if (tdo_val == "none"){
        google_speak_none(voice_tmp,voice_lang);
        return;
      }
      if (tdo_val == "input_disp"){
        if (voice_lang == "en"){
          if (tdo_id == "none"){tdo_ch = "unknown"}
          voice_tmp = "Ok" + tdo_ch + "is" + tdo_id;
        } else {
          if (tdo_id == "none"){tdo_id = "不明"}
          voice_tmp = tdo_ch + "は" + tdo_id + "です";
　　　　　　　}
　　      speak_main(voice_tmp,voice_lang);
        return;
      }
      if (tdo_val != "none" && tdo_val != "half"){
        send_do(tdo_ch,tdo_val,tdo_time);
        google_speak(voice_tmp,voice_lang);
        return;
      }
      if (tdo_val == "half"){
        switch (tdo_id){
          case 'do0':
            do_id_val = di2json.do0;
           break;
          case 'do1':
            do_id_val = di2json.do1;
            break;
          case 'do2':
            do_id_val = di2json.do2;
            break;
          case 'do3':
            do_id_val = di2json.do3;
            break;
          case 'do4':
            do_id_val = di2json.do4;
            break;
          case 'do5':
            do_id_val = di2json.do5;
            break;
          case 'do6':
            do_id_val = di2json.do6;
            break;
          case 'do7':
            do_id_val = di2json.do7;
            break;
          case 'to1':
            do_id_val = di2json.to1;
            break;
          case 'to2':
            do_id_val = di2json.to2;
            break;
          case 'to3':
            do_id_val = di2json.to3;
            break;
        }
        if (do_id_val == "none" || do_id_val == "low"){
          tdo_val_tmp = "1";
          if (voice_lang == "en"){
            voice_tmp = "turn on the" + voice_str;
          }
          else {
            voice_tmp = voice_str + "、おん";
          }
        }
        if (do_id_val == "high"){
          tdo_val_tmp = "0";
          if (voice_lang == "en"){
            voice_tmp = "turn off the" + voice_str;
          }
          else {
            voice_tmp = voice_str + "、おふ";
          }
        }
        send_do(tdo_ch,tdo_val_tmp,tdo_time);
        google_speak(voice_tmp,voice_lang);
        return;
      }
      google_speak(voice_src,voice_lang);
    }, // end of ajax success
    error: function(di2json){
      if (voice_lang == "en"){
        voice_tmp = "Sory,Server time-out do not acept the" + voice_src;
      }
      else {
        voice_tmp = "サーバータイムアウトの為、" + voice_src　+ "、が実行できませんでした。";
      }
      speak_main(voice_tmp,voice_lang);
      return;
    }
  }); // end of ajax
  return;
}
// Digital output process
function update_do(do_sel,results_voice){
  var tdo_val = "none";
  var tdo_ch = "none";
  var tdo_time = "none";
  switch (do_sel){
    case 'dosel_0':
      tdo_val = $('#dosel_0 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "0";
      tdo_time = $('#don_time_0').val();
      break;
    case 'dosel_1':
      tdo_val = $('#dosel_1 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "1";
      tdo_time = $('#don_time_1').val();
      break;
    case 'dosel_2':
      tdo_val = $('#dosel_2 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "2";
      tdo_time = $('#don_time_2').val();
      break;
    case 'dosel_3':
      tdo_val = $('#dosel_3 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "3";
      tdo_time = $('#don_time_3').val();
      break;
    case 'dosel_4':
      tdo_val = $('#dosel_4 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "4";
      tdo_time = $('#don_time_4').val();
      break;
    case 'dosel_5':
      tdo_val = $('#dosel_5 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "5";
      tdo_time = $('#don_time_5').val();
      break;
    case 'dosel_6':
      tdo_val = $('#dosel_6 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "6";
      tdo_time = $('#don_time_6').val();
      break;
    case 'dosel_7':
      tdo_val = $('#dosel_7 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "7";
      tdo_time = $('#don_time_7').val();
      break;
    case 'irkitdo_0':
      tdo_val = $('#dosel_8 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "8";
      tdo_time = $('#don_time_8').val();
      break;
    case 'irkitdo_1':
      tdo_val = $('#dosel_9 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "9";
      tdo_time = $('#don_time_8').val();
      break;
    case 'irkitdo_2':
      tdo_val = $('#dosel_10 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "10";
      tdo_time = $('#don_time_10').val();
      break;
    case 'irkitdo_3':
      tdo_val = $('#dosel_11 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "11";
      tdo_time = $('#don_time_11').val();
      break;
    case 'irkitdo_4':
      tdo_val = $('#dosel_12 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "12";
      tdo_time = $('#don_time_12').val();
      break;
    case 'irkitdo_5':
      tdo_val = $('#dosel_13 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "13";
      tdo_time = $('#don_time_13').val();
      break;
    case 'tocosdo_1':
      tdo_val = $('#dosel_14 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "14";
      tdo_time = $('#don_time_14').val();
      break;
    case 'tocosdo_2':
      tdo_val = $('#dosel_15 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "15";
      tdo_time = $('#don_time_15').val();
      break;
    case 'tocosdo_3':
      tdo_val = $('#dosel_16 option:selected').val();
      if (tdo_val == "none") return;
      tdo_ch = "16";
      tdo_time = $('#don_time_16').val();
      break;
    case 'voice_sel':
      if (results_voice === undefined){
        var results_voice = $('#voice_val').val();
      if (results_voice == "") return;
      }
      voice_do(do_sel,results_voice);
      break;
  }
  if (tdo_val != "none"){
    send_do(tdo_ch,tdo_val,tdo_time);
  }
  return;
}
function update_di(item){
  function color_set(di_v, disp_v, val_v){
    var color_bg;
    var color_font;
    var Update_di_Timer = 0;
    switch (val_v){
    case "none":
      color_bg = "#A9A9A9";
      color_font = "#000000";
      break;
    case "high":
      color_bg = "#DA0B00";
      color_font = "#F0FFFF";
      break;
    case "low":
      color_bg = "#008000";
      color_font = "#F0FFFF";
      break;
    case "good":
      color_bg = "#008000";
      color_font = "#F0FFFF";
      break;
    case "average":
      color_bg = "#FFFF00";
      color_font = "#F0FFFF";
      break;
    case "little_bad":
      color_bg = "#FF8000";
      color_font = "#F0FFFF";
      break;
    case "bad":
      color_bg = "#FF0000";
      color_font = "#F0FFFF";
      break;
    case "worse":
      color_bg = "#610B38";
      color_font = "#F0FFFF";
      break;
    case "very_bad":
      color_bg = "#000000";
      color_font = "#F0FFFF";
      break;
    default:
      color_bg = "#A9A9A9";
      color_font = "#F0FFFF";
      break;
    }
    if (val_v == "high" || val_v == "low" || val_v == "none"){
      $(di_v).html(disp_v + '<INPUT TYPE="text" size="1" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:36px;text-align:center" VALUE="' + val_v + '">');
    } else if (document.getElementById("s_phone_temp_hum") != null){
      $(di_v).html('<INPUT TYPE="text" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:200px;text-align:center" VALUE="' + val_v + '">');
    } else {
      $(di_v).html(disp_v + '<INPUT TYPE="text" size="1" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:36px;text-align:center" VALUE="' + val_v + '">');
    }
  }
  function diocount(t_item, t_reset, t_update, t_count, t_log_id){
    if (t_update === undefined) t_update = "";
    $(t_item).html('Count:<INPUT TYPE="text" readonly style="width:36px;text-align:right;" VALUE="' + t_count + '">&nbsp;' + t_reset + ' ～ ' + t_update);
    var disp_log = '#' + t_log_id;
    $(disp_log).html('<input type="button" value="Log display" onclick="disp_di_log(\'' + t_log_id + '\');"/>');
  }
  function s_phone_do_color_set(di_v,disp_v,val_v,do_item,do_alias){
    var color_bg;
    var color_font;
    switch (val_v){
      case "high":
      color_bg = "#DA0B00";
      color_font = "#F0FFFF";
      break;
    case "low":
      color_bg = "#008000";
      color_font = "#F0FFFF";
      break;
    default:
      color_bg = "#A9A9A9";
      color_font = "#F0FFFF";
      break;
    }
      $(di_v).html('<INPUT TYPE="button" id="' + do_alias + '" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:240px;text-align:center" VALUE="' + disp_v + '" onClick=s_phone_update_do("' + do_item + '")><BR><BR>');
  }
  function s_phone_di_color_set(di_v, disp_v, val_v){
    var color_bg;
    var color_font;
    switch (val_v){
      case "high":
        color_bg = "#DA0B00";
        color_font = "#F0FFFF";
        break;
      case "low":
        color_bg = "#008000";
        color_font = "#F0FFFF";
        break;
    default:
      color_bg = "#A9A9A9";
      color_font = "#F0FFFF";
      break;
    }
      $(di_v).html('<INPUT TYPE="text" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:260px;text-align:center" VALUE="' + disp_v + '"><BR><BR>');
  }
  function s_phone_di_graph(di_span,di_name,di_val,di_cgi,br,wid){
    var color_bg = "#E6E6E6";
    var color_font = "#000000";
    if (wid === undefined) wid = 400;
    if (br == "" || br === undefined){
      $(di_span).html('<INPUT TYPE="button" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:' + wid + "px" + ';text-align:center" VALUE="' + di_name + " " + di_val + '" onClick="window.open(\'' + di_cgi + '\',\'\',\'width=600,height=200\')"><BR><BR>');
    } else {
      $(di_span).html('<INPUT TYPE="button" readonly style="color:' + color_font + ';background-color:' + color_bg + ';width:' + wid + "px" + ';text-align:center" VALUE="' + di_name + " " + di_val + '" onClick="window.open(\'' + di_cgi + '\',\'\',\'width=600,height=200\')">');
    }
  }
  $(function(){
    var val = "";
    var val_alias = "";
    var do_item　= "";
    var do_alias = "";
    var reset = "";
    var count = "";
    var update = "";
    var log_id = "";
//    $("#disp_menu5").text("Loading...");
    var dummy = (new Date().getTime()).toString();
    var di_get_url = ".di_read_data.json?" + dummy;
    $.ajax({
      url: di_get_url,
      dataType: "json",
      type: "get",
      cache: true,
      async: true,
      timeout : 3000,
      success: function(di2json, status){
        val = status;
        $("#disp_menu5").text("di2json status " + val);
          if (di2json.hp_timestamp){
            var date = new Date() ; // now time
            var now_m = date.getTime();
            var fstamp_m = di2json.hp_timestamp * 1000;
            var time_lag = Math.abs(now_m - fstamp_m) ; // mili second calculation of time
            var reload_tm = unsmapho_reload_tm + 2000;
 //             console.log({time_lag});
 //             console.log({reload_tm});
            if (time_lag <  reload_tm) {
              setTimeout(function(){
                location.reload();
              },reload_tm);
            }
          }
          if (di2json.voice_req){
            val = di2json.voice_req;
            if (val == "high"){
              startWebVoiceRecognition();
            } else {
                if (val != "none"){
                  var tmp = (val.match(/[^@＠]+/));
                  if (tmp.index == 1){
                    var voice_lang_val = $("#voice_lang").val();
                    if (voice_lang_val === null) voice_lang_val = ja;
                    val = tmp[0];
                    speak_exec(val,voice_lang_val);
                  } else {
                      update_do("voice_sel",val);
                }
              }
            }
          }
          if (di2json.do0){
            val = di2json.do0;
            color_set("#do_0", "Output1", val);
            if (di2json.alias_do0){
              val_alias = di2json.alias_do0;
              if (val_alias != "none"){
                do_item = "dosel_0";
                do_alias = "alias_do_0";
                s_phone_do_color_set("#s_phone_do0",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do1){
            val = di2json.do1;
            color_set("#do_1", "Output2", val);
            if (di2json.alias_do1){
              val_alias = di2json.alias_do1;
              if (val_alias != "none"){
                do_item = "dosel_1";
                do_alias = "alias_do_1";
                s_phone_do_color_set("#s_phone_do1",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do2){
            val = di2json.do2;
            color_set("#do_2", "Output3", val);
            if (di2json.alias_do2){
              val_alias = di2json.alias_do2;
              if (val_alias != "none"){
                do_item = "dosel_2";
                do_alias = "alias_do_2";
                s_phone_do_color_set("#s_phone_do2",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do3){
            val = di2json.do3;
            color_set("#do_3", "Output4", val);
            if (di2json.alias_do3){
              val_alias = di2json.alias_do3;
              if (val_alias != "none"){
                do_item = "dosel_3";
                do_alias = "alias_do_3";
                s_phone_do_color_set("#s_phone_do3",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do4){
            val = di2json.do4;
            color_set("#do_4", "Output5", val);
            if (di2json.alias_do4){
              val_alias = di2json.alias_do4;
              if (val_alias != "none"){
                do_item = "dosel_4";
                do_alias = "alias_do_4";
                s_phone_do_color_set("#s_phone_do4",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do5){
            val = di2json.do5;
            color_set("#do_5", "Output6", val);
            if (di2json.alias_do5){
              val_alias = di2json.alias_do5;
              if (val_alias != "none"){
                do_item = "dosel_5";
                do_alias = "alias_do_5";
                s_phone_do_color_set("#s_phone_do5",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do6){
            val = di2json.do6;
            color_set("#do_6", "Output7", val);
            if (di2json.alias_do6){
              val_alias = di2json.alias_do6;
              if (val_alias != "none"){
                do_item = "dosel_6";
                do_alias = "alias_do_6";
                s_phone_do_color_set("#s_phone_do6",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.do7){
            val = di2json.do7;
            color_set("#do_7", "Output8", val);
            if (di2json.alias_do7){
              val_alias = di2json.alias_do7;
              if (val_alias != "none"){
                do_item = "dosel_7";
                do_alias = "alias_do_7";
                s_phone_do_color_set("#s_phone_do7",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.irdata_0){
            val = di2json.irdata_0;
            disp_irdata("#irdata_0",val);
            if (di2json.alias_do8){
              val_alias = di2json.alias_do8;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_0";
                do_alias = "alias_do_8";
                s_phone_do_color_set("#s_phone_do8",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.irdata_1){
            val = di2json.irdata_1;
            disp_irdata("#irdata_1",val);
            if (di2json.alias_do9){
             val_alias = di2json.alias_do9;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_1";
                 do_alias = "alias_do_9";
                s_phone_do_color_set("#s_phone_do9",val_alias,val,do_item,do_alias);
             }
            }
          }
          if (di2json.irdata_2){
            val = di2json.irdata_2;
            disp_irdata("#irdata_2",val);
            if (di2json.alias_do10){
              val_alias = di2json.alias_do10;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_2";
                do_alias = "alias_do_10";
                s_phone_do_color_set("#s_phone_do10",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.irdata_3){
            val = di2json.irdata_3;
            disp_irdata("#irdata_3",val);
            if (di2json.alias_do11){
              val_alias = di2json.alias_do11;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_3";
                do_alias = "alias_do_11";
                s_phone_do_color_set("#s_phone_do11",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.irdata_4){
            val = di2json.irdata_4;
            disp_irdata("#irdata_4",val);
            if (di2json.alias_do12){
              val_alias = di2json.alias_do12;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_4";
                do_alias = "alias_do_12";
                s_phone_do_color_set("#s_phone_do12",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.irdata_5){
            val = di2json.irdata_5;
            disp_irdata("#irdata_5",val);
            if (di2json.alias_do13){
              val_alias = di2json.alias_do13;
              if (val_alias != "none"){
                if (val == "Ready"){
                  val = "low"
                } else {val = "none"}
                do_item = "irkitdo_5";
                do_alias = "alias_do_13";
                s_phone_do_color_set("#s_phone_do13",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.to1){
            val = di2json.to1;
            color_set("#do_14","TO1",val);
            if (di2json.alias_do14){
              val_alias = di2json.alias_do14;
              if (val_alias != "none"){
                do_item = "tocosdo_1";
                do_alias = "alias_do_14";
                s_phone_do_color_set("#s_phone_do14",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.to2){
            val = di2json.to2;
            color_set("#do_15","TO2",val);
            if (di2json.alias_do15){
              val_alias = di2json.alias_do15;
              if (val_alias != "none"){
                do_item = "tocosdo_2";
                do_alias = "alias_do_15";
                s_phone_do_color_set("#s_phone_do15",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.to3){
            val = di2json.to3;
            color_set("#do_16","TO3",val);
            if (di2json.alias_do16){
              val_alias = di2json.alias_do16;
              if (val_alias != "none"){
                do_item = "tocosdo_3";
                do_alias = "alias_do_16";
                s_phone_do_color_set("#s_phone_do16",val_alias,val,do_item,do_alias);
              }
            }
          }
          if (di2json.i2ctemp){
            val = di2json.i2ctemp;
            if (val == "none"){
              $("#i2ctemp").text(val);
              $("#i2c_temp_disp").text("");
              $("#i2c_hum_disp").text("");
              $("#s_phone_i2c_temp_disp").text("");
              $("#s_phone_i2c_hum_disp").text("");
              $("#s_phone_tocos_hum_disp").text("");
            }
            else {
              var tval = new Array(3);
              tval[1] = val.temp;
              tval[2] = val.hum;
              tval[3] = val.date + " " + val.temp + " " + val.hum;
              $("#i2c_temp_val").text(tval[1]);
              $("#i2c_hum_val").text(tval[2]);
              $("#i2ctemp").text(tval[3]);
              if (document.getElementById("s_phone_tocos_temp_hum") != null){
                $("#s_phone_tocos_temp_hum").html('<HR><input type="button" NAME="i2c_temp_disp" VALUE="' + tval[1] + '" onClick="window.open(\'./i2c_temp_disp_day.cgi\',\'\',\'width=600,height=200\')">&nbsp;&nbsp;&nbsp;&nbsp;<input type="button" NAME="i2c_hum_disp" VALUE="' + tval[2] + '"  onClick="window.open(\'./i2c_hum_disp_day.cgi\',\'\',\'width=600,height=200\')">');
              } else {
                $("#i2c_temp_disp").html('<input id="i2c_temp_disp" type="button" NAME="i2c_temp_disp" VALUE="Twlite Temperature Graph" onClick="window.open(\'./i2c_temp_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="i2c_temp_disp_day" type="button" NAME="i2c_temp_disp_day" VALUE="Twlite Temperature Day Graph " onClick="window.open(\'./i2c_temp_disp_day.cgi\',\'\',\'width=600,height=200\')">') ;
                $("#i2c_hum_disp").html('<input id="i2c_hum_disp" type="button" NAME="i2c_hum_disp" VALUE="Twlite Humidity Graph" onClick="window.open(\'./i2c_hum_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="i2c_hum_disp_day" type="button" NAME="i2c_hum_disp_day" VALUE="Twlite Humidity Day Graph" onClick="window.open(\'./i2c_hum_disp_day.cgi\',\'\',\'width=600,height=200\')">');
                s_phone_di_graph("#s_phone_i2c_temp_disp","Twlite Temp",tval[1],"i2c_temp_disp_day.cgi");
                s_phone_di_graph("#s_phone_i2c_hum_disp","Twlite Hum",tval[2],"i2c_hum_disp_day.cgi");
              }
            }
          }
          if (di2json.di0){
            val = di2json.di0;
            color_set("#di_0", "Input1",  val);
            color_set("#menu90di_0", "",  val);
            color_set("#menu91di_0", "",  val);
            color_set("#menu100di_0", "",  val);
            color_set("#menu101di_0", "",  val);
            if (di2json.alias_di0){
              val_alias = di2json.alias_di0;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di0",val_alias,val);
              }
            }
          }
          if (di2json.di1){
            val = di2json.di1;
            color_set("#di_1", "Input2", val);
            color_set("#menu90di_1", "",  val);
            color_set("#menu91di_1", "",  val);
            color_set("#menu100di_1", "",  val);
            color_set("#menu101di_1", "",  val);
            if (di2json.alias_di1){
              val_alias = di2json.alias_di1;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di1",val_alias,val);
              }
            }
          }
          if (di2json.di2){
            val = di2json.di2;
            color_set("#di_2", "Input3", val);
            color_set("#menu90di_2", "",  val);
            color_set("#menu91di_2", "",  val);
            color_set("#menu100di_2", "",  val);
            color_set("#menu101di_2", "",  val);
            if (di2json.alias_di2){
              val_alias = di2json.alias_di2;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di2",val_alias,val);
              }
            }
          }
          if (di2json.di3){
            val = di2json.di3;
            color_set("#di_3", "Input4", val);
            color_set("#menu90di_3", "",  val);
            color_set("#menu91di_3", "",  val);
            color_set("#menu100di_3", "",  val);
            color_set("#menu101di_3", "",  val);
            if (di2json.alias_di3){
              val_alias = di2json.alias_di3;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di3",val_alias,val);
              }
            }
          }
          if (di2json.di4){
            val = di2json.di4;
            color_set("#di_4", "Input5", val);
            color_set("#menu90di_4", "",  val);
            color_set("#menu91di_4", "",  val);
            color_set("#menu100di_4", "",  val);
            color_set("#menu101di_4", "",  val);
            if (di2json.alias_di4){
              val_alias = di2json.alias_di4;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di4",val_alias,val);
              }
            }
          }
          if (di2json.di5){
            val = di2json.di5;
            color_set("#di_5", "Input6", val);
            color_set("#menu90di_5", "",  val);
            color_set("#menu91di_5", "",  val);
            color_set("#menu100di_5", "",  val);
            color_set("#menu101di_5", "",  val);
            if (di2json.alias_di5){
              val_alias = di2json.alias_di5;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di5",val_alias,val);
              }
            }
          }
          if (di2json.di6){
            val = di2json.di6;
            color_set("#di_6", "Input7", val);
            color_set("#menu90di_6", "",  val);
            color_set("#menu91di_6", "",  val);
            color_set("#menu100di_6", "",  val);
            color_set("#menu101di_6", "",  val);
            if (di2json.alias_di6){
              val_alias = di2json.alias_di6;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di6",val_alias,val);
              }
            }
          }
          if (di2json.di7){
            val = di2json.di7;
            color_set("#di_7", "Input8", val);
            color_set("#menu90di_7", "",  val);
            color_set("#menu91di_7", "",  val);
            color_set("#menu100di_7", "",  val);
            color_set("#menu101di_7", "",  val);
            if (di2json.alias_di7){
              val_alias = di2json.alias_di7;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di7",val_alias,val);
              }
            }
          }
          if (di2json.ti1){
            val = di2json.ti1;
            color_set("#di_8", "TI1", val);
            color_set("#menu90di_8", "",  val);
            color_set("#menu91di_8", "",  val);
            color_set("#menu100di_8", "",  val);
            color_set("#menu101di_8", "",  val);
            if (di2json.alias_di8){
              val_alias = di2json.alias_di8;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di8",val_alias,val);
              }
            }
          }
          if (di2json.ti2){
            val = di2json.ti2;
            color_set("#di_9", "TI2", val);
            color_set("#menu90di_9", "",  val);
            color_set("#menu91di_9", "",  val);
            color_set("#menu100di_9", "",  val);
            color_set("#menu101di_9", "",  val);
            if (di2json.alias_di9){
              val_alias = di2json.alias_di9;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di9",val_alias,val);
              }
            }
          }
          if (di2json.ti3){
            val = di2json.ti3;
            color_set("#di_10", "TI3", val);
            color_set("#menu90di_10", "",  val);
            color_set("#menu91di_10", "",  val);
            color_set("#menu100di_10", "",  val);
            color_set("#menu101di_10", "",  val);
            if (di2json.alias_di10){
              val_alias = di2json.alias_di10;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di10",val_alias,val);
              }
            }
          }
          if (di2json.ai2di1){
            val = di2json.ai2di1;
            color_set("#di_12", "AI1", val);
            color_set("#menu90di_12", "",  val);
            color_set("#menu91di_12", "",  val);
            color_set("#menu100di_12", "",  val);
            color_set("#menu101di_12", "",  val);
            if (di2json.alias_di12){
              val_alias = di2json.alias_di12;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di12",val_alias,val);
              }
            }
          }
          if (di2json.ai2di2){
            val = di2json.ai2di2;
            color_set("#di_13", "AI2", val);
            color_set("#menu90di_13", "",  val);
            color_set("#menu91di_13", "",  val);
            color_set("#menu100di_13", "",  val);
            color_set("#menu101di_13", "",  val);
            if (di2json.alias_di13){
              val_alias = di2json.alias_di13;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di13",val_alias,val);
              }
            }
          }
          if (di2json.ai2di3){
            val = di2json.ai2di3;
            color_set("#di_14", "AI3", val);
            color_set("#menu90di_14", "",  val);
            color_set("#menu91di_14", "",  val);
            color_set("#menu100di_14", "",  val);
            if (di2json.alias_di14){
              val_alias = di2json.alias_di14;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di14",val_alias,val);
              }
            }
            color_set("#menu101di_14", "",  val);
          }
          if (di2json.ai2di4){
            val = di2json.ai2di4;
            color_set("#di_15", "AI4", val);
            color_set("#menu90di_15", "",  val);
            color_set("#menu91di_15", "",  val);
            color_set("#menu100di_15", "",  val);
            color_set("#menu101di_15", "",  val);
            if (di2json.alias_di15){
              val_alias = di2json.alias_di15;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di15",val_alias,val);
              }
            }
          }
          if (di2json.ai2di5){
            val = di2json.ai2di5;
            color_set("#di_16", "CPU Temperature", val);
            color_set("#menu90di_16", "",  val);
            color_set("#menu91di_16", "",  val);
            color_set("#menu100di_16", "",  val);
            color_set("#menu101di_16", "",  val);
            if (di2json.alias_di16){
              val_alias = di2json.alias_di16;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di16",val_alias,val);
              }
            }
          }
          if (di2json.ai2di6){
            val = di2json.ai2di6;
            color_set("#di_17", "GPIO Temperature", val);
            color_set("#menu90di_17", "",  val);
            color_set("#menu91di_17", "",  val);
            color_set("#menu100di_17", "",  val);
            color_set("#menu101di_17", "",  val);
            if (di2json.alias_di17){
              val_alias = di2json.alias_di17;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di17",val_alias,val);
              }
            }
          }
          if (di2json.ai2di7){
            val = di2json.ai2di7;
            color_set("#di_18", "GPIO Humidity", val);
            color_set("#menu90di_18", "",  val);
            color_set("#menu91di_18", "",  val);
            color_set("#menu100di_18", "",  val);
            color_set("#menu101di_18", "",  val);
            if (di2json.alias_di18){
              val_alias = di2json.alias_di18;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di18",val_alias,val);
              }
            }
          }
          if (di2json.ai2di8){
            val = di2json.ai2di8;
            color_set("#di_19", "Twlite Temperature", val);
            color_set("#menu90di_19", "",  val);
            color_set("#menu91di_19", "",  val);
            color_set("#menu100di_19", "",  val);
            color_set("#menu101di_19", "",  val);
            if (di2json.alias_di19){
              val_alias = di2json.alias_di19;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di19",val_alias,val);
              }
            }
          }
          if (di2json.ai2di9){
            val = di2json.ai2di9;
            color_set("#di_20", "Twlite Humidity", val);
            color_set("#menu90di_20", "",  val);
            color_set("#menu91di_20", "",  val);
            color_set("#menu100di_20", "",  val);
            color_set("#menu101di_20", "",  val);
            if (di2json.alias_di20){
              val_alias = di2json.alias_di20;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di20",val_alias,val);
              }
            }
          }
          if (di2json.ai2di10){
            val = di2json.ai2di10;
            color_set("#di_21", "GPIO Pressure", val);
            color_set("#menu90di_21", "",  val);
            color_set("#menu91di_21", "",  val);
            color_set("#menu100di_21", "",  val);
            color_set("#menu101di_21", "",  val);
            if (di2json.alias_di21){
              val_alias = di2json.alias_di21;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di21",val_alias,val);
              }
            }
          }
          if (di2json.ai2di11){
            val = di2json.ai2di11;
            color_set("#di_22", "GPIO Gas", val);
            color_set("#menu90di_22", "",  val);
            color_set("#menu91di_22", "",  val);
            color_set("#menu100di_22", "",  val);
            color_set("#menu101di_22", "",  val);
            if (di2json.alias_di22){
              val_alias = di2json.alias_di22;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di22",val_alias,val);
              }
            }
          }
          if (di2json.ai2di12){
            val = di2json.ai2di12;
            color_set("#di_23", "IAQ Sample", val);
            color_set("#menu90di_23", "",  val);
            color_set("#menu91di_23", "",  val);
            color_set("#menu100di_23", "",  val);
            color_set("#menu101di_23", "",  val);
            if (di2json.alias_di23){
              val_alias = di2json.alias_di23;
              if (val_alias != "none"){
                s_phone_di_color_set("#s_phone_di23",val_alias,val);
              }
            }
          }
          if (di2json.vai1){
            val = di2json.vai1;
            if (val != "-1"){
              $("#vai_1").text(val+"mv");
              $("#vai_1_graph").html('&nbsp;<input id="vai_1_graph" type="button" NAME="vai_1_graph" VALUE="Analog input-1 Graph" onClick="window.open(\'./vai1_graph_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="vai_1_graph_day" type="button" NAME="vai_1_graph_day" VALUE="Analog input-1 Day Graph" onClick="window.open(\'./vai1_graph_disp_day.cgi\',\'\',\'width=600,height=200\')">');
              val = val + "mv";
              s_phone_di_graph("#s_phone_vai_1_graph","Twlite AI1",val, "vai1_graph_disp_day.cgi");
            }
            else {
              $("#vai_1").text("");
              $("#vai_1_graph").text("");
              $("#s_phone_vai_1_graph").text("");
            }
          }
          if (di2json.vai2){
            val = di2json.vai2;
            if (val != "-1"){
              $("#vai_2").text(val+"mv");
              $("#vai_2_graph").html('&nbsp;<input id="vai_2_graph" type="button" NAME="vai_2_graph" VALUE="Analog input-2 Graph" onClick="window.open(\'./vai2_graph_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="vai_2_graph_day" type="button" NAME="vai_2_graph_day" VALUE="Analog input-2 Day Graph" onClick="window.open(\'./vai2_graph_disp_day.cgi\',\'\',\'width=600,height=200\')">');
              val = val + "mv";
              s_phone_di_graph("#s_phone_vai_2_graph","Twlite AI2",val, "vai2_graph_disp_day.cgi");
            }
            else {
              $("#vai_2").text("");
              $("#vai_2_graph").text("");
              $("#s_phone_vai_2_graph").text("");
            }
          }
          if (di2json.vai3){
            val = di2json.vai3;
            if (val != "-1"){
              $("#vai_3").text(val+"mv");
              $("#vai_3_graph").html('&nbsp;<input id="vai_3_graph" type="button" NAME="vai_3_graph" VALUE="Analog input-2 Graph" onClick="window.open(\'./vai2_graph_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="vai_3_graph_day" type="button" NAME="vai_3_graph_day" VALUE="Analog input-2 Day Graph" onClick="window.open(\'./vai2_graph_disp_day.cgi\',\'\',\'width=600,height=200\')">');
              val = val + "mv";
              s_phone_di_graph("#s_phone_vai_3_graph","Twlite AI3",val, "vai3_graph_disp_day.cgi");
            }
            else {
              $("#vai_3").text("");
              $("#vai_3_graph").text("");
              $("#s_phone_vai_3_graph").text("");
            }
          }
          if (di2json.vai4){
            val = di2json.vai4;
            if (val != "-1"){
              $("#vai_4").text(val+"mv");
              $("#vai_4_graph").html('&nbsp;<input id="vai_4_graph" type="button" NAME="vai_4_graph" VALUE="Analog input-2 Graph" onClick="window.open(\'./vai2_graph_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input id="vai_4_graph_day" type="button" NAME="vai_4_graph_day" VALUE="Analog input-2 Day Graph" onClick="window.open(\'./vai2_graph_disp_day.cgi\',\'\',\'width=600,height=200\')">');
              val = val + "mv";
              s_phone_di_graph("#s_phone_vai_4_graph","Twlite AI4",val, "vai4_graph_disp_day.cgi");
            }
            else {
              $("#vai_4").text("");
              $("#vai_4_graph").text("");
              $("#s_phone_vai_4_graph").text("");
            }
          }
          // Display of GPIO Temperature&Humidity
          if (di2json.gpio_i2c){
            val = di2json.gpio_i2c;
            if ( val == "none") {
              $("#gpio_i2c").text(val);
              $("#gpio_temp_val").text("");
              $("#gpio_hum_val").text("");
              $("#gpio_pres_val").text("");
              $("#gpio_gas_val").text("");
              $("#gpio_iaq_val").text("");
              $("#s_phone_temp_hum").text("");
              if (di2json.date){
                var tdate = di2json.date;
                var date_val = tdate.split(":");
                tdate = date_val[0] + ":" + date_val[1] + " " + date_val[3];
              } else {
                date_val =   "Server-Timeout";
              }
              $("#s_phone_temp_hum").html('<input type="button" NAME="time_disp" VALUE="' + tdate + '">');
            }
            else {
              var tval = new Array(6);
              tval[1] = val.temp;
              tval[2] = val.hum;
              tval[3] = val.pres;
              tval[4] = val.gas;
              tval[5] = val.iaq;
              $("#gpio_temp_val").text(tval[1]);
              $("#gpio_hum_val").text(tval[2]);
              $("#gpio_pres_val").text(tval[3]);
              $("#gpio_gas_val").text(tval[4]);
              $("#gpio_iaq_val").text(val.iaq)
              var iaq_val = tval[5];
              var iaq_color = "none";
              if (iaq_val != "none"){
                if ( iaq_val <= 50) {
                  iaq_color = "good";
                } else if ( iaq_val <= 100) {
                  iaq_color = "average";
                } else if ( iaq_val <= 150) {
                  iaq_color = "little_bad";
                } else if ( iaq_val <= 200) {
                  iaq_color = "bad";
                } else if ( iaq_val <= 300) {
                  iaq_color = "worse";
                } else if ( iaq_val > 300) {
                  iaq_color = "very_bad";
                }
              }
              if (document.getElementById("s_phone_temp_hum") != null){
                if (di2json.date){
                  var tdate = di2json.date;
                  var date_val = tdate.split(":");
                  tdate = date_val[0] + ":" + date_val[1] + " " + date_val[3];
                } else {
                  date_val =   "Server-Timeout";
                }
                $("#s_phone_temp_hum").html('<input type="button" NAME="time_disp" VALUE="' + tdate + '"><BR>');
                var wid = "240";
                if (tval[1] != "none") s_phone_di_graph("#s_phone_gpio_temp_graph","",tval[1],"gpio_temp_disp_day.cgi","nobr",wid);
                if (tval[2] != "none") s_phone_di_graph("#s_phone_gpio_hum_graph","",tval[2],"gpio_hum_disp_day.cgi","nobr",wid);
                if (tval[3] != "none") s_phone_di_graph("#s_phone_gpio_pres_graph","",tval[3],"gpio_pres_disp_day.cgi","nobr",wid);
                if (tval[4] != "none") s_phone_di_graph("#s_phone_gpio_gas_graph","",tval[4],"gpio_gas_disp_day.cgi","nobr",wid);
                if (tval[5] != "none") s_phone_di_graph("#s_phone_gpio_iaq_graph","IAQ ",tval[5],"gpio_iaq_disp_day.cgi","nobr",wid);
                if (iaq_val != "none") color_set("#gpio_iaq_val",iaq_val,iaq_color);
                if (tval[1] != "none") s_phone_di_graph("#s_phone_gpio_csv","CSV Data","", "gpiorrdtoolfetch.cgi","",wid);
              } else {
                val = di2json.cpu_temp;
                $("#gpio_temp_graph").html('&nbsp;<input type="button" NAME="gpio_temp_disp" VALUE="Temperature Graph" onClick="window.open(\'./gpio_temp_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_temp_disp" VALUE="Temperature Day Graph" onClick="window.open(\'./gpio_temp_disp_day.cgi\',\'\',\'width=600,height=200\')">');
                $("#gpio_hum_graph").html('&nbsp;<input type="button" NAME="gpio_hum_disp" VALUE="Humidity Graph" onClick="window.open(\'./gpio_hum_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_hum_disp" VALUE="Humidity Day Graph" onClick="window.open(\'./gpio_hum_disp_day.cgi\',\'\',\'width=600,height=200\')">');
                $("#gpio_pres_graph").html('&nbsp;<input type="button" NAME="gpio_pres_disp" VALUE="Pressure Graph" onClick="window.open(\'./gpio_pres_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_pres_disp" VALUE="Pressure Day Graph" onClick="window.open(\'./gpio_pres_disp_day.cgi\',\'\',\'width=600,height=200\')">');
                $("#gpio_gas_graph").html('&nbsp;<input type="button" NAME="gpio_gas_disp" VALUE="Gas Graph" onClick="window.open(\'./gpio_gas_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_gas_disp" VALUE="Gas Day Graph" onClick="window.open(\'./gpio_gas_disp_day.cgi\',\'\',\'width=600,height=200\')">');
                $("#gpio_iaq_graph").html('&nbsp;<input type="button" NAME="gpio_iaq_disp" VALUE="IAQ Graph" onClick="window.open(\'./gpio_iaq_disp.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_iaq_disp" VALUE="IAQ Day Graph" onClick="window.open(\'./gpio_iaq_disp_day.cgi\',\'\',\'width=600,height=200\')">&nbsp;<input type="button" NAME="gpio_iaq_disp" VALUE="Last hour CSV data" onClick="window.open(\'./gpiorrdtoolfetch.cgi\',\'\',\'width=400,height=600\')">');
                if (iaq_val != "none") color_set("#gpio_iaq_val",iaq_val,iaq_color);
                if (val != undefined) s_phone_di_graph("#s_phone_cpu_temp_graph","CPU Temp",val, "cpu_temp_disp.cgi");
                if (tval[1] != "none") s_phone_di_graph("#s_phone_gpio_temp_graph","GPIO Temp",tval[1], "gpio_temp_disp_day.cgi");
                if (tval[2] != "none") s_phone_di_graph("#s_phone_gpio_hum_graph","GPIO Hum",tval[2], "gpio_hum_disp_day.cgi");
                if (tval[3] != "none") s_phone_di_graph("#s_phone_gpio_pres_graph","GPIO Pres",tval[3], "gpio_pres_disp_day.cgi");
                if (tval[4] != "none") s_phone_di_graph("#s_phone_gpio_gas_graph","GPIO Gas",tval[4], "gpio_gas_disp_day.cgi");
                if (tval[5] != "none") s_phone_di_graph("#s_phone_gpio_iaq_graph","GPIO IAQ",tval[5], "gpio_iaq_disp_day.cgi");
                if (tval[1] != "none") s_phone_di_graph("#s_phone_gpio_csv","GPIO CSV Data","", "gpiorrdtoolfetch.cgi");
              }
            }
          }
          // Date and time display
          if (di2json.date){
            var tval = di2json.date;
            val = tval.split(":");
            val = val[0] + ":" + val[1];
            $("#disp_menu5").text("Server-Synchronized at " + val);
            $("#server_time").text(val);
          }
        // Cpu temperature display
          if (di2json.cpu_temp){
            val = di2json.cpu_temp;
            $("#cpu_temp").text(val);
          }
           if (di2json.dio0high){
            item = "#menu90ct_0";
            reset = di2json.dio0high.reset;
            log_id = "dio0high";
            if (di2json.dio0high.update) update = di2json.dio0high.update;
            count = di2json.dio0high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_0";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio1high){
            item = "#menu90ct_1";
            reset = di2json.dio1high.reset;
            log_id = "dio1high";
            update = di2json.dio1high.update;
            count = di2json.dio1high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_1";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio2high){
            item = "#menu90ct_2";
            reset = di2json.dio2high.reset;
            log_id = "dio2high";
            update = di2json.dio2high.update;
            count = di2json.dio2high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_2";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio3high){
            item = "#menu90ct_3";
            reset = di2json.dio3high.reset;
            log_id = "dio3high";
            update = di2json.dio3high.update;
            count = di2json.dio3high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_3";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio4high){
            item = "#menu90ct_4";
            reset = di2json.dio4high.reset;
            log_id = "dio4high";
            update = di2json.dio4high.update;
            count = di2json.dio4high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_4";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio5high){
            item = "#menu90ct_5";
            reset = di2json.dio5high.reset;
            log_id = "dio5high";
            update = di2json.dio5high.update;
            count = di2json.dio5high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_5";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio6high){
            item = "#menu90ct_6";
            reset = di2json.dio6high.reset;
            log_id = "dio6high";
            update = di2json.dio6high.update;
            count = di2json.dio6high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_6";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio7high){
            item = "#menu90ct_7";
            reset = di2json.dio7high.reset;
            log_id = "dio7high";
            update = di2json.dio7high.update;
            count = di2json.dio7high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_7";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio8high){
            item = "#menu90ct_8";
            reset = di2json.dio8high.reset;
            log_id = "dio8high";
            update = di2json.dio8high.update;
            count = di2json.dio8high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_8";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio9high){
            item = "#menu90ct_9";
            reset = di2json.dio9high.reset;
            log_id = "dio9high";
            update = di2json.dio9high.update;
            count = di2json.dio9high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_9";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio10high){
            item = "#menu90ct_10";
            reset = di2json.dio10high.reset;
            log_id = "dio10high";
            update = di2json.dio10high.update;
            count = di2json.dio10high.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu100ct_10";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio0low){
            item = "#menu91ct_0";
            reset = di2json.dio0low.reset;
            log_id = "dio0low";
            update = di2json.dio0low.update;
            count = di2json.dio0low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_0";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio1low){
            item = "#menu91ct_1";
            reset = di2json.dio1low.reset;
            log_id = "dio1low";
            update = di2json.dio1low.update;
            count = di2json.dio1low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_1";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio2low){
            item = "#menu91ct_2";
            reset = di2json.dio2low.reset;
            log_id = "dio2low";
            update = di2json.dio2low.update;
            count = di2json.dio2low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_2";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio3low){
            item = "#menu91ct_3";
            reset = di2json.dio3low.reset;
            log_id = "dio3low";
            update = di2json.dio3low.update;
            count = di2json.dio3low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_3";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio4low){
            item = "#menu91ct_4";
            reset = di2json.dio4low.reset;
            log_id = "dio4low";
            update = di2json.dio4low.update;
            count = di2json.dio4low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_4";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio5low){
            item = "#menu91ct_5";
            reset = di2json.dio5low.reset;
            log_id = "dio5low";
            update = di2json.dio5low.update;
            count = di2json.dio5low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_5";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio6low){
            item = "#menu91ct_6";
            reset = di2json.dio6low.reset;
            log_id = "dio6low";
            update = di2json.dio6low.update;
            count = di2json.dio6low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_6";
            diocount(item, reset, update, count, log_id);
          }
          if (di2json.dio7low){
            item = "#menu91ct_7";
            reset = di2json.dio7low.reset;
            log_id = "dio7low";
            update = di2json.dio7low.update;
            count = di2json.dio7low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_7";
            diocount(item, reset, update, count, log_id);
          }
        if (di2json.dio8low){
            item = "#menu91ct_8";
            reset = di2json.dio8low.reset;
            log_id = "dio8low";
            update = di2json.dio8low.update;
            count = di2json.dio8low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_8";
            diocount(item, reset, update, count, log_id);
          }
        if (di2json.dio9low){
            item = "#menu91ct_9";
            reset = di2json.dio9low.reset;
            log_id = "dio9low";
            update = di2json.dio9low.update;
            count = di2json.dio9low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_9";
            diocount(item, reset, update, count, log_id);
          }
        if (di2json.dio10low){
            item = "#menu91ct_10";
            reset = di2json.dio10low.reset;
            log_id = "dio10low";
            update = di2json.dio10low.update;
            count = di2json.dio10low.count;
            diocount(item, reset, update, count, log_id);
            item = "#menu101ct_10";
            diocount(item, reset, update, count, log_id);
          }
// IRKit IP Read & set
        if (di2json.irkit_ip){
            var ip = di2json.irkit_ip;
            $("#irkit_ip").text(ip);
          }
        else {
            $("#irkit_ip").text("");
        }
// Disp Sound File
        if (di2json.disp_sound_0){
          val = di2json.disp_sound_0;
          $("#disp_sound_0").text(val);
        }
        if (di2json.disp_sound_1){
          val = di2json.disp_sound_1;
          $("#disp_sound_1").text(val);
        }
        if (di2json.disp_sound_2){
          val = di2json.disp_sound_2;
          $("#disp_sound_2").text(val);
        }
        if (di2json.disp_sound_3){
          val = di2json.disp_sound_3;
          $("#disp_sound_3").text(val);
        }
        if (di2json.disp_sound_4){
          val = di2json.disp_sound_4;
          $("#disp_sound_4").text(val);
        }
        if (di2json.disp_sound_5){
          val = di2json.disp_sound_5;
          $("#disp_sound_5").text(val);
        }
        if (di2json.disp_sound_6){
          val = di2json.disp_sound_6;
          $("#disp_sound_6").text(val);
        }
        if (di2json.disp_sound_7){
          val = di2json.disp_sound_7;
          $("#disp_sound_7").text(val);
        }
        if (di2json.disp_sound_8){
          val = di2json.disp_sound_8;
          $("#disp_sound_8").text(val);
        }
        if (di2json.disp_sound_9){
          val = di2json.disp_sound_9;
          $("#disp_sound_9").text(val);
        }
       },
       error: function(di2json){
          $("#disp_menu5").text("Server-Timeout");
       }
    });
  });
  if (item == "onload"){
    var browser_os = "unknown";
    var tmp_os = navigator.platform;
    var tmp_ua = navigator.userAgent;
    // ios
    if (tmp_os == "iPhone") browser_os = "iPhone";
    if (tmp_os == "iPad") browser_os = "iPad";
    // Android
    if (tmp_os == "Android") browser_os = "Android";
    if (tmp_ua.indexOf("Android") > 0) browser_os = "Android";
    //Windows, MacOS
/*    var tmp_ua = navigator.userAgent;
    if (tmp_ua.indexOf("iPhone") > 0) browser_os = "iPhone";
    if (tmp_ua.indexOf("Windows") > 0) browser_os = "Windows";
    if (tmp_ua.indexOf("Mac") > 0) browser_os = "MacOS";
    if (tmp_ua.indexOf("Android") > 0) browser_os = "Android";
    if (tmp_ua.indexOf("Linux") > 0) browser_os = "Linux";
*/
    if (browser_os == "iPhone" || browser_os == "Android"){
      if (document.getElementById("s_phone_temp_hum") != null){
        Update_di_Timer = setTimeout("update_di('onload')",unsmapho_reload_tm); // Temp&Hum Disp information update time (milliseconds)
      } else {
        Update_di_Timer = setTimeout("update_di('onload')",smapho_reload_tm); // DIO information update time (milliseconds)
      }
    } else {
      Update_di_Timer = setTimeout("update_di('onload')",unsmapho_reload_tm); // DIO information update time (milliseconds)
    }
  }
  if (item == "onunload"){
    clearTimeout(Update_di_Timer);
  }
}

function kana_ck(){
/* Phonetic check */
  var str = document.iform.FuriganaText.value;
  if(str.match(/[^ぁ-んァ-ン　\s]+/)){
    alert("Phonetic, please Input only hiragana, katakana");
    return -1;
  }
  return 1;
}

function keypress(){
/*  Enter key invalid */
    if(window.event.keyCode == 13){
        return false;
    }
    return true;
}
window.document.onkeydown= keypress;

function alpha_ck(str){
/* Single-byte English characters check */
  if(str.match(/[^A-Za-z\s.-]+/)){
    alert(str + "←" + "Please Input only single-byte English characters.");
    return -1;
  }
  return 1;
}

function num_ck(str){
/* number　check */
  if(str.match(/[^0-9]+/)){
    alert(str + "←" + "Non-numeric is entered");
    return -1;
  }
  return 1;
}

function input_date_ck(str){
/*  Check of Input date format:2013/10/01 */
    if(!str.match(/^\d{4}\/\d{2}\/\d{2}$/)){
        return false;
    }
    var myYear = str.substr(0, 4) - 0;
    var myMonth = str.substr(5, 2) - 1;
    var myDay = str.substr(8, 2) - 0;
    if(myMonth >= 0 && myMonth <= 11 && myDay >= 1 && myDay <= 31){
        var myDt = new Date(myYear, myMonth, myDay);
        if(isNaN(myDt)){
            return false;
        }else if(myDt.getFullYear() == myYear && myDt.getMonth() == myMonth && myDt.getDate() == myDay){
            return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}

function input_timer_ck(str){
/* Check of Input time */
  if(!str.match(/^\d{2}\:\d{2}:\d{2}$/)){
    return false;
  }
  var myHour = str.substr(0, 2) - 0;
  var myMinutes = str.substr(3, 2) - 0;
  var mySecond = str.substr(6, 2) - 0;
  if(myHour >= 0 && myHour <= 24 && myMinutes >= 0 && myMinutes <= 59 && mySecond >= 0 && mySecond <= 59){
    return true;
  }else{
    return false;
  }
}

function hostname_ck(str){
/* Check and alphanumeric */
  if(str.match(/[^0-9A-Za-z\s.-]+/)){
    alert(str + "←" + "Unauthorized content has been Input")
    return -1
  }
  return 1
}

function num_alpha_ck(str){
/* Alphanumeric check */
  if(str.match(/[^0-9A-Za-z]+/)){
    alert(str + "←" + "Please be Input only alphanumeric characters");
    return -1;
  }
  return 1;
}

function password_ck(pass){
  if (pass.match(/[^0-9A-Za-z]+/)){
    alert("Please be Input password is alphanumeric only");
    return -1;
  }
  return 1;
}

function user_ck(user,pass){
/* Alphanumeric check */
  if(user.length == 0 || pass.length == 0){
    alert("User name or password is blank");
    return -1;
  }
  if (pass == "*") return -1;
  if(user.match(/[^0-9A-Za-z]+/)){
    alert("Please be Input only with the user name is alphanumeric");
    return -1;
  }
  if(pass.match(/[^0-9A-Za-z]+/)){
    alert("Please be Input password is alphanumeric only");
    return -1;
  }
  return 1;
}

function timer_ck(str){
/* DIO on-time check */
  var error_ct = 0;
  var check = 0;
  check++;
  if (num_ck(str) == -1){
    error_ct++;
  }
  else {
    if (str <= 0 || str > 300000){
    　alert(str + "←" + "Time column I can Input up to blank or 1-300000");
      error_ct++;
    }
  }
  if (error_ct >0){
    return -1;
  }
  return 1;
}

function ipaddr_ck(str){
/* ipaddress check */
  var ck_pattern = /^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$/;
  if (! str.match(ck_pattern)){
    alert(str + "←" + "The IP address is incorrect") ;
    return -1;
  }
  ip = str.split(".");
　for(i = 0; i < 4; i++){
　  if((ip[i] < 0) || (ip[i] > 255)){
  　  alert(str + "←" + "The IP address is incorrect") ;
      return -1;
    }
  }
  return 1;
}

function mail_ck(str){
/* mail address check */
//  var ck_pattern = /[!#-9A-~]+@+[0-9A-Za-z.-]+.+[^.]$/;
  var ck_pattern = /[!#-9A-~]+[0-9A-Za-z.-]+.+[^.]$/;
  if(! str.match(ck_pattern)){
    alert(str + "←" + "There is an error in the e-mail address");
    return -1;
  }
  return 1;
}

function url_ck(str){
/* URL address check */
  var url_pattern = /^(((ht|f)tp(s?))\:\/\/)([-_.!~*'()a-zA-Z0-9;/?:@&=+$,%#]+)?(\/\S*)?$/
  if (! url_pattern.test(str)){
    alert(str + "←" + "There is an error in the URL");
  return -1;
  }
  return 1;
}

function menu4_ck(button_id,disp_id){
/* upload sound file & check */
  var check = 0;
  var error_ct = 0;
  var call_cgi = "none";
  var file_name = "none";
  var max_filesize = 2048 * 1024; /* 2MB */
  var formdata = new FormData($('#menu4').get(0));
  switch (button_id){
    case 'menu4_sound_del':
      call_cgi = 'sound_del.cgi';
      break;
    case 'menu4_sound_0':
      file_name = document.menu4.sound_file_0.value;
      if (file_name != ""){
        disp_id = disp_sound_0;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_0')[0].files[0].size);
        formdata.append('sound_file_0',$('#sound_file_0')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_1':
      file_name = document.menu4.sound_file_1.value;
      if (file_name != ""){
        disp_id = disp_sound_1;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_1')[0].files[0].size);
        formdata.append('sound_file_1',$('#sound_file_1')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_2':
      file_name = document.menu4.sound_file_2.value;
      if (file_name != ""){
        disp_id = disp_sound_2;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_2')[0].files[0].size);
        formdata.append('sound_file_2',$('#sound_file_2')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_3':
      file_name = document.menu4.sound_file_3.value;
      if (file_name != ""){
        disp_id = disp_sound_3;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_3')[0].files[0].size);
        formdata.append('sound_file_3',$('#sound_file_3')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_4':
      file_name = document.menu4.sound_file_4.value;
      if (file_name != ""){
        disp_id = disp_sound_4;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_4')[0].files[0].size);
        formdata.append('sound_file_4',$('#sound_file_4')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_5':
      file_name = document.menu4.sound_file_5.value;
      if (file_name != ""){
        disp_id = disp_sound_5;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_5')[0].files[0].size);
        formdata.append('sound_file_5',$('#sound_file_5')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_6':
      file_name = document.menu4.sound_file_6.value;
      if (file_name != ""){
        disp_id = disp_sound_6;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_6')[0].files[0].size);
        formdata.append('sound_file_6',$('#sound_file_6')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_7':
      file_name = document.menu4.sound_file_7.value;
      if (file_name != ""){
        disp_id = disp_sound_7;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_7')[0].files[0].size);
        formdata.append('sound_file_7',$('#sound_file_7')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_8':
      file_name = document.menu4.sound_file_8.value;
      if (file_name != ""){
        disp_id = disp_sound_8;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_8')[0].files[0].size);
        formdata.append('sound_file_8',$('#sound_file_8')[0].files[0]);
      } else {
        return false;
      }
      break;
    case 'menu4_sound_9':
      file_name = document.menu4.sound_file_9.value;
      if (file_name != ""){
        disp_id = disp_sound_9;
        call_cgi = 'sound_upload.cgi';
        formdata.append('size',$('#sound_file_9')[0].files[0].size);
        formdata.append('sound_file_9',$('#sound_file_9')[0].files[0]);
      } else {
        return false;
      }
      break;
    default:
      return false;
      break;
  }
  if (call_cgi == 'sound_del.cgi'){
    var disp_info = '#' + disp_id;
    file_name = $(disp_info).html();
    if (file_name != ""){
      $(disp_info).text("File Delete in progress.");
      $.ajax({
        type: "get",
        url: call_cgi,
        timeout : 10000,
        dataType: "text",
        async: true,
        data: disp_id + '=' + file_name,
        success: function(){
          $(disp_info).text("File Delete Success!");
          return;
        },
        error: function(){
          $(disp_info).text("File Delete faile!");
          return;
        }
      });
    }
    return false;
  }
  if (call_cgi == "sound_upload.cgi"){
    if (formdata.get('size') > max_filesize){
      var text_max_filesize = max_filesize / 1024 + "KB";
      $(disp_id).text("File size is less than " + text_max_filesize);
      formdata.dalete;
      return false;
    }
    setInterval(function(){
      $(disp_id).text("File upload in progress.");
      $(disp_id).fadeOut('slow',function(){
        $(this).fadeIn('slow');
      });
    },2000);
    $.ajax({
      type: "POST",
      url: call_cgi,
      timeout : 30000,
      dataType: "html",
      data: formdata,
      cache       : false,
      contentType : false,
      processData : false,
      success: function(){
        $(disp_id).text("File Upload Success!");
      },
      error: function(){
        $(disp_id).text("File Upload faile!");
      }
    });
    setInterval(function(){
      var jump_location = "wait_for.cgi?"
      location.href=jump_location;
      return;
    },30000);
  }
}

function menu5_ck(){
/* dio timer check */
  var i = 0;
  var error_ct = 0;
  var array_time = new Array(8);
  array_time[0] = document.menu5.don_time_0.value;
  array_time[1] = document.menu5.don_time_1.value;
  array_time[2] = document.menu5.don_time_2.value;
  array_time[3] = document.menu5.don_time_3.value;
  var array_slice = new Array(12);
  array_slice[0] = document.menu5.slice_ai_12.value;
  array_slice[1] = document.menu5.slice_ai_13.value;
  array_slice[2] = document.menu5.slice_ai_14.value;
  array_slice[3] = document.menu5.slice_ai_15.value;
  array_slice[4] = document.menu5.slice_ai_16.value;
  array_slice[5] = document.menu5.slice_ai_17.value;
  array_slice[6] = document.menu5.slice_ai_18.value;
  array_slice[7] = document.menu5.slice_ai_19.value;
  array_slice[8] = document.menu5.slice_ai_20.value;
  array_slice[9] = document.menu5.slice_ai_21.value;
  array_slice[10] = document.menu5.slice_ai_22.value;
  array_slice[11] = document.menu5.slice_ai_23.value;
  array_slice[12] = document.menu5.slice_ai_24.value;
  document.addEventListener('click', function (e){
    var target = e.target;
    var form = target.form;
    if (target.tagName === 'INPUT' && target.type === 'button'){
      switch (target.id){
        case 'menu5_jikkou':
          break;
        case 'menu5_dio':
          return false;
          break;
        default:
          return false;
          break;
      }
    }
  }, false);
  for (i=0 ; i <=3 ; i++){
    if (array_time[i] != ""){
      if (timer_ck(array_time[i]) == -1) error_ct++;
    }
  }
  for (i=0 ; i <=12 ; i++){
    if (array_slice[i] != ""){
      if (num_ck(array_slice[i]) == -1) error_ct++;
    }
  }
  if (error_ct != 0){
    return false;
  }
  document.getElementById("menu5_form").submit();
}

function i2c_temp_disp(){
　　window.open("./i2c_temp_disp.cgi" , "i2c_temp_disp" , "width=240,height=160");
　　return;
}

function i2c_hum_disp(){
　　window.open("./i2c_hum_disp.cgi" , "i2c_hum_disp" , "width=240,height=160");
　　return;
}

function menu6_ck(){
/* IP address check */
  var check = 0;
  var error_ct = 0;
  var array_ip = new Array(3);
  var array_mail = new Array(3);
  array_ip[0] = document.menu6.ip_0.value;
  array_ip[1] = document.menu6.ip_1.value;
  array_ip[2] = document.menu6.ip_2.value;
  array_ip[3] = document.menu6.ip_3.value;
  for (var i=0 ; i <=3 ; i++){
    if (array_ip[i] != ""){
      check++;
      if (ipaddr_ck(array_ip[i]) == -1){
        error_ct++;
      }
    }
  }
  if (error_ct == 0 && check > 0){
    document.getElementById("menu6_form").submit();
  }
  return false;
}

function menu7_ck (){
/* ip addresses and e-mail address check */
  var check = 0;
  var error_ct = 0;
  var array_ip = new Array(3);
  var array_mail = new Array(3);
  var array_reg = new Array(3);
  array_reg[0] = document.menu7.reg_0.value;
  array_reg[1] = document.menu7.reg_1.value;
  array_reg[2] = document.menu7.reg_2.value;
  array_reg[3] = document.menu7.reg_3.value;
  array_ip[0] = document.menu7.ip_0.value;
  array_ip[1] = document.menu7.ip_1.value;
  array_ip[2] = document.menu7.ip_2.value;
  array_ip[3] = document.menu7.ip_3.value;
  array_mail[0] = document.menu7.mail_0.value;
  array_mail[1] = document.menu7.mail_1.value;
  array_mail[2] = document.menu7.mail_2.value;
  array_mail[3] = document.menu7.mail_3.value;
  for (var i=0 ; i <=3 ; i++){
    if (array_reg[i] == "del" && array_ip[i] != ""){
      check++;
      if (ipaddr_ck(array_ip[i]) == -1){
        error_ct++;
      }
    }
    if (array_reg[i] == "reg" && array_ip[i] != "" && array_mail[i] != ""){
      check++;
      if (ipaddr_ck(array_ip[i]) == -1){
        error_ct++;
      }
      if (mail_ck(array_mail[i]) == -1){
        error_ct++;
      }
    }
    if (array_reg[i] == "reg" &&  array_ip[i] != "" && array_mail[i] == ""){
      alert("here are places that have not been Input to the email address field");
      return false;
    }
    if (array_reg[i] == "reg" && array_ip[i] == "" && array_mail[i] != ""){
      alert("There are places that have not been Input to IP address column");
      return false;
    }
  }
  if (error_ct == 0 && check > 0){
    document.getElementById("menu7_form").submit();
  }
  return false;
}

function menu8_ck (){
/* ip address and phone number check */
  var check = 0;
  var error_ct = 0;
  var array_ip = new Array(3);
  var array_tel = new Array(3);
  var array_reg = new Array(3);
  array_reg[0] = document.menu8.reg_0.value;
  array_reg[1] = document.menu8.reg_1.value;
  array_reg[2] = document.menu8.reg_2.value;
  array_reg[3] = document.menu8.reg_3.value;
  array_ip[0] = document.menu8.ip_0.value;
  array_ip[1] = document.menu8.ip_1.value;
  array_ip[2] = document.menu8.ip_2.value;
  array_ip[3] = document.menu8.ip_3.value;
  array_tel[0] = document.menu8.tel_0.value;
  array_tel[1] = document.menu8.tel_1.value;
  array_tel[2] = document.menu8.tel_2.value;
  array_tel[3] = document.menu8.tel_3.value;
  for (var i=0 ; i <=3 ; i++){
    if (array_reg[i] == "del" && array_ip[i] != ""){
      check++;
      if (ipaddr_ck(array_ip[i]) == -1){
        error_ct++;
      }
    }
    if (array_reg[i] == "reg" && array_ip[i] != "" && array_tel[i] != ""){
      check++;
      if (ipaddr_ck(array_ip[i]) == -1){
        error_ct++;
      }
    }
    if (array_reg[i] == "reg" && array_ip[i] != "" && array_tel[i] == ""){
      alert("There are places that have not been Input to the phone number field");
      return false;
    }
    if (array_reg[i] == "reg" && array_ip[i] == "" && array_tel[i] != ""){
      alert("There are places that have not been Input to IP address column");
      return false;
    }
  }
  if(error_ct == 0 && check > 0){
    document.getElementById("menu8_form").submit();
  }
  return false;
}

function menu9_ck (){
/* DIO-1 management of check */
  var check = 0;
  var error_ct = 0;
  var array_change = new Array(22);
  var array_tel = new Array(22);
  var array_mail = new Array(22);
  var array_mail_msg = new Array(22);
  var array_di_act = new Array(22);
  var array_don_time = new Array(22);
  array_change[0] = document.menu9.di_change_reg_0.value;
  array_change[1] = document.menu9.di_change_reg_1.value;
  array_change[2] = document.menu9.di_change_reg_2.value;
  array_change[3] = document.menu9.di_change_reg_3.value;
  array_change[8] = document.menu9.di_change_reg_8.value;
  array_change[9] = document.menu9.di_change_reg_9.value;
  array_change[10] = document.menu9.di_change_reg_10.value;
  array_change[11] = document.menu9.di_change_reg_11.value;
  array_change[12] = document.menu9.di_change_reg_12.value;
  array_change[13] = document.menu9.di_change_reg_13.value;
  array_change[14] = document.menu9.di_change_reg_14.value;
  array_change[19] = document.menu9.di_change_reg_19.value;
  array_change[20] = document.menu9.di_change_reg_20.value;
  array_change[21] = document.menu9.di_change_reg_21.value;
  array_tel[0] = document.menu9.di_tel_0.value;
  array_tel[1] = document.menu9.di_tel_1.value;
  array_tel[2] = document.menu9.di_tel_2.value;
  array_tel[3] = document.menu9.di_tel_3.value;
  array_tel[8] = document.menu9.di_tel_8.value;
  array_tel[9] = document.menu9.di_tel_9.value;
  array_tel[10] = document.menu9.di_tel_10.value;
  array_tel[11] = document.menu9.di_tel_11.value;
  array_tel[12] = document.menu9.di_tel_12.value;
  array_tel[13] = document.menu9.di_tel_13.value;
  array_tel[14] = document.menu9.di_tel_14.value;
  array_tel[19] = document.menu9.di_tel_19.value;
  array_tel[20] = document.menu9.di_tel_20.value;
  array_tel[21] = document.menu9.di_tel_21.value;
  array_mail[0] = document.menu9.di_mail_0.value;
  array_mail[1] = document.menu9.di_mail_1.value;
  array_mail[2] = document.menu9.di_mail_2.value;
  array_mail[3] = document.menu9.di_mail_3.value;
  array_mail[8] = document.menu9.di_mail_8.value;
  array_mail[9] = document.menu9.di_mail_9.value;
  array_mail[10] = document.menu9.di_mail_10.value;
  array_mail[11] = document.menu9.di_mail_11.value;
  array_mail[12] = document.menu9.di_mail_12.value;
  array_mail[13] = document.menu9.di_mail_13.value;
  array_mail[14] = document.menu9.di_mail_14.value;
  array_mail[19] = document.menu9.di_mail_19.value;
  array_mail[20] = document.menu9.di_mail_20.value;
  array_mail[21] = document.menu9.di_mail_21.value;
  array_mail_msg[0] = document.menu9.di_mail_message_0.value;
  array_mail_msg[1] = document.menu9.di_mail_message_1.value;
  array_mail_msg[2] = document.menu9.di_mail_message_2.value;
  array_mail_msg[3] = document.menu9.di_mail_message_3.value;
  array_mail_msg[8] = document.menu9.di_mail_message_8.value;
  array_mail_msg[9] = document.menu9.di_mail_message_9.value;
  array_mail_msg[10] = document.menu9.di_mail_message_10.value;
  array_mail_msg[11] = document.menu9.di_mail_message_11.value;
  array_mail_msg[12] = document.menu9.di_mail_message_12.value;
  array_mail_msg[13] = document.menu9.di_mail_message_13.value;
  array_mail_msg[14] = document.menu9.di_mail_message_14.value;
  array_mail_msg[19] = document.menu9.di_mail_message_19.value;
  array_mail_msg[20] = document.menu9.di_mail_message_20.value;
  array_mail_msg[21] = document.menu9.di_mail_message_21.value;
  array_di_act[0] = document.menu9.di_act_0.value;
  array_di_act[1] = document.menu9.di_act_1.value;
  array_di_act[2] = document.menu9.di_act_2.value;
  array_di_act[3] = document.menu9.di_act_3.value;
  array_di_act[8] = document.menu9.di_act_8.value;
  array_di_act[9] = document.menu9.di_act_9.value;
  array_di_act[10] = document.menu9.di_act_10.value;
  array_di_act[11] = document.menu9.di_act_11.value;
  array_di_act[12] = document.menu9.di_act_12.value;
  array_di_act[13] = document.menu9.di_act_13.value;
  array_di_act[14] = document.menu9.di_act_14.value;
  array_di_act[19] = document.menu9.di_act_19.value;
  array_di_act[20] = document.menu9.di_act_20.value;
  array_di_act[21] = document.menu9.di_act_21.value;
  array_don_time[0] = document.menu9.don_time_0.value;
  array_don_time[1] = document.menu9.don_time_1.value;
  array_don_time[2] = document.menu9.don_time_2.value;
  array_don_time[3] = document.menu9.don_time_3.value;
  array_don_time[8] = document.menu9.don_time_8.value;
  array_don_time[9] = document.menu9.don_time_9.value;
  array_don_time[10] = document.menu9.don_time_10.value;
  array_don_time[11] = document.menu9.don_time_11.value;
  array_don_time[12] = document.menu9.don_time_12.value;
  array_don_time[13] = document.menu9.don_time_13.value;
  array_don_time[14] = document.menu9.don_time_14.value;
  array_don_time[19] = document.menu9.don_time_19.value;
  array_don_time[20] = document.menu9.don_time_20.value;
  array_don_time[21] = document.menu9.don_time_21.value;
  for (var i=0 ; i<=21 ;i++){
    if (array_change[i] == "reg"){
      if (array_mail[i] != ""){
        if (array_di_act[i] == "mail" || array_di_act[i] == "mail_message" || array_di_act[i] == "web_camera_still" || array_di_act[i] == "web_camera_video" || array_di_act[i] == "mod_camera_still" || array_di_act[i] == "mod_camera_video"){
          check++;
          if (mail_ck(array_mail[i]) == -1){
            error_ct++;
          }
        }
      }
      if (array_mail[i] == ""){
        if (array_di_act[i] == "mail" || array_di_act[i] == "mail_message" || array_di_act[i] == "web_camera_still" || array_di_act[i] == "web_camera_video" || array_di_act[i] == "mod_camera_still" || array_di_act[i] == "mod_camera_video"){
          alert("here are places that have not been Input to the email address field");
          error_ct++;
          return false;
        }
      }
      if (array_di_act[i] == "mail_message"){
        if (array_mail_msg[i] != "") check++;
        else {
          error_ct++;
          alert("mail message box empty");
          return false;
        }
      }
      if (array_di_act[i] == "phone" && array_tel[i] != ""){
        check++;
        if (num_ck(array_tel[i]) == -1) error_ct++ ;
      }
      if (array_di_act[i] == "phone" && array_tel[i] ==""){
        alert("There are places that have not been Input to the phone number field");
        return false;
      }
      if (array_di_act[i] != "phone" && array_di_act[i] != "mail"){
        check++;
        if (array_don_time[i] != "" && timer_ck(array_don_time[i]) == -1){
          error_ct++;
        }
      }
    }
    if (array_change[i] == "del" || array_change[i] == "clr"){
      check++;
    }
  }
  if(error_ct == 0 && check > 0){
    document.getElementById("menu9_form").submit();
  }
  return false;
}
function menu10_ck (){
/* DIO-2 management of check */
  var check = 0;
  var error_ct = 0;
  var array_change = new Array(22);
  var array_tel = new Array(22);
  var array_mail = new Array(22);
  var array_mail_msg = new Array(22);
  var array_di_act = new Array(22);
  var array_don_time = new Array(22);
  array_change[0] = document.menu10.di_change_reg_0.value;
  array_change[1] = document.menu10.di_change_reg_1.value;
  array_change[2] = document.menu10.di_change_reg_2.value;
  array_change[3] = document.menu10.di_change_reg_3.value;
  array_change[8] = document.menu10.di_change_reg_8.value;
  array_change[9] = document.menu10.di_change_reg_9.value;
  array_change[10] = document.menu10.di_change_reg_10.value;
  array_change[11] = document.menu10.di_change_reg_11.value;
  array_change[12] = document.menu10.di_change_reg_12.value;
  array_change[13] = document.menu10.di_change_reg_13.value;
  array_change[14] = document.menu10.di_change_reg_14.value;
  array_change[19] = document.menu10.di_change_reg_19.value;
  array_change[20] = document.menu10.di_change_reg_20.value;
  array_change[21] = document.menu10.di_change_reg_21.value;
  array_tel[0] = document.menu10.di_tel_0.value;
  array_tel[1] = document.menu10.di_tel_1.value;
  array_tel[2] = document.menu10.di_tel_2.value;
  array_tel[3] = document.menu10.di_tel_3.value;
  array_tel[8] = document.menu10.di_tel_8.value;
  array_tel[9] = document.menu10.di_tel_9.value;
  array_tel[10] = document.menu10.di_tel_10.value;
  array_tel[11] = document.menu10.di_tel_11.value;
  array_tel[12] = document.menu10.di_tel_12.value;
  array_tel[13] = document.menu10.di_tel_13.value;
  array_tel[14] = document.menu10.di_tel_14.value;
  array_tel[19] = document.menu10.di_tel_19.value;
  array_tel[20] = document.menu10.di_tel_20.value;
  array_tel[21] = document.menu10.di_tel_21.value;
  array_mail[0] = document.menu10.di_mail_0.value;
  array_mail[1] = document.menu10.di_mail_1.value;
  array_mail[2] = document.menu10.di_mail_2.value;
  array_mail[3] = document.menu10.di_mail_3.value;
  array_mail[8] = document.menu10.di_mail_8.value;
  array_mail[9] = document.menu10.di_mail_9.value;
  array_mail[10] = document.menu10.di_mail_10.value;
  array_mail[11] = document.menu10.di_mail_11.value;
  array_mail[12] = document.menu10.di_mail_12.value;
  array_mail[13] = document.menu10.di_mail_13.value;
  array_mail[14] = document.menu10.di_mail_14.value;
  array_mail[19] = document.menu10.di_mail_19.value;
  array_mail[20] = document.menu10.di_mail_20.value;
  array_mail[21] = document.menu10.di_mail_21.value;
  array_mail_msg[0] = document.menu10.di_mail_message_0.value;
  array_mail_msg[1] = document.menu10.di_mail_message_1.value;
  array_mail_msg[2] = document.menu10.di_mail_message_2.value;
  array_mail_msg[3] = document.menu10.di_mail_message_3.value;
  array_mail_msg[8] = document.menu10.di_mail_message_8.value;
  array_mail_msg[9] = document.menu10.di_mail_message_9.value;
  array_mail_msg[10] = document.menu10.di_mail_message_10.value;
  array_mail_msg[11] = document.menu10.di_mail_message_11.value;
  array_mail_msg[12] = document.menu10.di_mail_message_12.value;
  array_mail_msg[13] = document.menu10.di_mail_message_13.value;
  array_mail_msg[14] = document.menu10.di_mail_message_14.value;
  array_mail_msg[19] = document.menu10.di_mail_message_19.value;
  array_mail_msg[20] = document.menu10.di_mail_message_20.value;
  array_mail_msg[21] = document.menu10.di_mail_message_21.value;
  array_di_act[0] = document.menu10.di_act_0.value;
  array_di_act[1] = document.menu10.di_act_1.value;
  array_di_act[2] = document.menu10.di_act_2.value;
  array_di_act[3] = document.menu10.di_act_3.value;
  array_di_act[8] = document.menu10.di_act_8.value;
  array_di_act[9] = document.menu10.di_act_9.value;
  array_di_act[10] = document.menu10.di_act_10.value;
  array_di_act[11] = document.menu10.di_act_11.value;
  array_di_act[12] = document.menu10.di_act_12.value;
  array_di_act[13] = document.menu10.di_act_13.value;
  array_di_act[14] = document.menu10.di_act_14.value;
  array_di_act[19] = document.menu10.di_act_19.value;
  array_di_act[20] = document.menu10.di_act_20.value;
  array_di_act[21] = document.menu10.di_act_21.value;
  array_don_time[0] = document.menu10.don_time_0.value;
  array_don_time[1] = document.menu10.don_time_1.value;
  array_don_time[2] = document.menu10.don_time_2.value;
  array_don_time[3] = document.menu10.don_time_3.value;
  array_don_time[8] = document.menu10.don_time_8.value;
  array_don_time[9] = document.menu10.don_time_9.value;
  array_don_time[10] = document.menu10.don_time_10.value;
  array_don_time[11] = document.menu10.don_time_11.value;
  array_don_time[12] = document.menu10.don_time_12.value;
  array_don_time[13] = document.menu10.don_time_13.value;
  array_don_time[14] = document.menu10.don_time_14.value;
  array_don_time[19] = document.menu10.don_time_19.value;
  array_don_time[20] = document.menu10.don_time_20.value;
  array_don_time[21] = document.menu10.don_time_21.value;
  for (var i=0 ; i<=21 ;i++){
    if (array_change[i] == "reg"){
      if (array_mail[i] != ""){
        if (array_di_act[i] == "mail" || array_di_act[i] == "mail_message" || array_di_act[i] == "web_camera_still" || array_di_act[i] == "web_camera_video" || array_di_act[i] == "mod_camera_still" || array_di_act[i] == "mod_camera_video"){
          check++;
          if (mail_ck(array_mail[i]) == -1){
            error_ct++;
          }
        }
      }
      if (array_mail[i] == ""){
        if (array_di_act[i] == "mail" || array_di_act[i] == "mail_message" || array_di_act[i] == "web_camera_still" || array_di_act[i] == "web_camera_video" || array_di_act[i] == "mod_camera_still" || array_di_act[i] == "mod_camera_video"){
          alert("here are places that have not been Input to the email address field");
          error_ct++;
          return false;
        }
      }
     if (array_di_act[i] == "mail_message"){
        if (array_mail_msg[i] != "") check++;
        else {
          error_ct++;
          alert("mail message box empty");
          return false;
        }
      }
       if (array_di_act[i] == "phone" && array_tel[i] ==""){
        alert("There are places that have not been Input to the phone number field");
        return false;
      }
      if (array_di_act[i] == "phone" && array_tel[i] != ""){
        check++;
        if (num_ck(array_tel[i]) == -1) error_ct++ ;
      }
      if (array_di_act[i] == "phone" && array_tel[i] ==""){
        alert("There are places that have not been Input to the phone number field");
        return false;
      }
      if (array_di_act[i] != "phone" && array_di_act[i] != "mail"){
        check++;
        if (array_don_time[i] != "" && timer_ck(array_don_time[i]) == -1){
          error_ct++;
        }
      }
    }
    if (array_change[i] == "del" || array_change[i] == "clr"){
      check++;
    }
  }
  if(error_ct == 0 && check > 0){
    document.getElementById("menu10_form").submit();
  }
  return false;
}

function menu11_ck (){
  var check = 0;
  var error_ct = 0;
  var gmailuser = document.menu11.elements['gmailuser'].value;
  var gmailpassword = document.menu11.elements['gmailpassword'].value;
  var permitmail = document.menu11.elements['permitmail'].value;
  var keyword = document.menu11.elements['keyword'].value;
  var jitter = document.menu11.elements['jitter'].value;
  var looptime = document.menu11.elements['looptime'].value;
  var reg = document.menu11.elements['reg'].value;
  if (reg == "del"){
    var jump_location = "gmail_set.cgi?" + "reg=" + reg;
    location.href=jump_location;
    return;
  }
  if (gmailuser  != "*" && gmailuser  != ""){
    check++;
  }
  if (gmailpassword != "*" && gmailpassword != "") check++;
  if (permitmail != "*" && permitmail != "") check++;
  if (keyword != "*" && keyword !="") check++;
  if (looptime != ""){
    check++;
    if (num_ck(looptime) == -1) error_ct++;
  }
  if (jitter != ""){
    check++;
    if (num_ck(jitter) == -1) error_ct++;
  }
  if (error_ct == 0 && check == 6){
    var jump_location = "gmail_set.cgi?" + "gmailuser=" + gmailuser + "&gmailpassword=" + gmailpassword + "&gwebuser=" + "&permitmail=" + permitmail + "&keyword=" + keyword + "&jitter=" + jitter + "&looptime=" + looptime + "&reg=" + reg;
    location.href=jump_location;
    return;
  }
  else {
    alert("There is an error in the item or Input content that has not been Input");
    return false;
  }
}

function menu12_ck (){
  var check = 0;
  var error_ct = 0;
  var i = 0;
  var array_wget = new Array(10);
  for (i=0 ; i<9 ; i++){
    array_wget[i] = document.menu12.elements[i].value;
  }
  for (i=0 ; i<9 ; i++){
    var str = array_wget[i];
    switch(i){
      case 0:
      check++;
      　　str = array_wget[i];
        if (url_ck(str) == -1){
          error_ct++;
        }
       break;
      case 1:
        if (str < 0 || str > 10){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 2:
        break;
      case 3:
        if (str < 0 || str > 59){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 4:
        if (str < 0 || str > 23){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 5:
        if (str < 0 || str > 31){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 6:
        if (str < 0 || str > 12){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 7:
        if (str < 0 || str > 7){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
    }
  }
  if(error_ct == 0 && check > 0){
    document.getElementById("menu12_form").submit();
  }
  else {
    return false;
  }
}

function cron_ck(ck_array,ct){
  var check = 0;
  var error_ct = 0;
  var i = 0;
  var tmp_array = new Array(2);
  var j_array = new Array(2);
  for (i = 1 ; i < 10 ; i++){
    var str = ck_array[i];
    j_array = str.split("-");
    if (str != "*" && j_array.length != 2 && i > 1 && i < 10){
      if (num_ck(str) == -1){
        error_ct++;
        alert(str + "← There is an error in the Input");
      }
      else check++;
    }
    if (str != "*" && j_array.length == 2 && i > 1 && i < 10){
      if (num_ck(j_array[0]) == -1 ||  num_ck(j_array[1]) == -1){
        error_ct++;
        alert(str + "← There is an error in the Input");
      }
      else check++;
    }
    switch(i){
      case 1:
        if (str == "none"){
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        else check++;
        break;
      case 2:
        if (str != ""){
          if (timer_ck(str) == -1){
            error_ct++;
            alert(str + "← There is an error in the Input");
          }
          else check++;
        }
        else check++;
        break;
      case 3:
        if (str == "*") check++;
        else {
          tmp_array = str.split("-");
          if (tmp_array.length == 2){
            if (tmp_array[0] < 0 || tmp_array[0] > 59){
              error_ct++;
              alert(tmp_array[0] + "← There is an error in the Input");
            }
            else check++;
            if (tmp_array[1] < 1 || tmp_array[1] > 59){
              error_ct++;
              alert(tmp_array[1] + "← There is an error in the Input");
            }
            else check++;
          }
          else {
            if (str < 0 || str > 59){
              error_ct++;
              alert(str + "← There is an error in the Input");
            }
            else check++;
          }
        }
        break;
      case 4:
        if (str == "*" || str == "2" || str == "4" || str == "5" || str == "6" || str == "10" || str == "15" || str == "20" || str == "30"){
          check++;
        }
        else {
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 5:
        if (str == "*") check++;
        else {
          tmp_array = str.split("-");
          if (tmp_array.length == 2){
            if (tmp_array[0] < 0 || tmp_array[0] > 23){
              error_ct++;
              alert(tmp_array[0] + "← There is an error in the Input");
            }
            if (tmp_array[1] < 1 || tmp_array[1] > 23){
              error_ct++;
              alert(tmp_array[1] + "← There is an error in the Input");
            }
            else {
              if (str < 0 || str > 23){
                error_ct++;
                alert(str + "← There is an error in the Input");
              }
              else check++;
            }
          }
        }
        break;
      case 6:
        if (str == "*" || str == "2" || str == "4" || str == "6" || str == "8" || str == "12"){
          check++;
        }
        else  {
          error_ct++;
          alert(str + "← There is an error in the Input");
        }
        break;
      case 7:
        if (str == "*") check++;
        else {
          if (str < 0 || str > 31){
            error_ct++;
            alert(str + "← There is an error in the Input");
          }
          else check++;
        }
        break;
      case 8:
        if (str == "*") check++;
        else {
          if (str < 0 || str > 12){
            error_ct++;
            alert(str + "← There is an error in the Input");
          }
          else check++;
        }
        break;
      case 9:
        if (str == "*") check++;
        else {
          if (str < 0 || str > 7)　{
            error_ct++;
            alert(str + "← There is an error in the Input");
          }
          else check++;
        }
        break;
    }
  }
  if(error_ct == 0 && check > 0) return 1;
  else return -1;
}

function menu12sub_ck(){
  var check = 0;
  var error_ct = 0;
  var i = 0;
  var j = 0;
  var k_array = new Array(2);
  var n = 0;
  var array_cron = new Array(10);
  var str = "";
  n = document.menu12sub.elements.length;
  while (j < n-2){
    for (i = 0; i <=10 ; i++){
      str = document.menu12sub.elements[j].value;
      array_cron[i] = str;
      if (j == 0 && document.menu12sub.elements[10].value == "none"){
        j = 11;
        break;
      }
      if (j == 11 && document.menu12sub.elements[21].value == "none"){
        j = 22;
        break;
      }
      if (j == 22 && document.menu12sub.elements[32].value == "none"){
        j = 33;
        break;
      }
      if (j == 33 && document.menu12sub.elements[43].value == "none"){
        j = 44;
        break;
      }
      if (j == 44 && document.menu12sub.elements[54].value == "none"){
        j = 55;
        break;
      }
      if (j == 55 && document.menu12sub.elements[65].value == "none"){
        j = 66;
        break;
      }
      if (j == 66 && document.menu12sub.elements[76].value == "none"){
        j = 77;
        break;
      }
      if (j == 77 && document.menu12sub.elements[87].value == "none"){
        j = 88;
        break;
      }
      if (j == 88 && document.menu12sub.elements[98].value == "none"){
        j = 99;
        break;
      }
      if (j == 99 && document.menu12sub.elements[109].value == "none"){
        j = 110;
        break;
      }
      if (j == 110 && document.menu12sub.elements[120].value == "none"){
        j = 121;
        break;
      }
      if (j == 121 && document.menu12sub.elements[131].value == "none"){
        j = 132;
        break;
      }
      if (j == 132 && document.menu12sub.elements[142].value == "none"){
        j = 143;
        break;
      }
      if (j == 143 && document.menu12sub.elements[153].value == "none"){
        j = 154;
        break;
      }
      if (j == 154 && document.menu12sub.elements[164].value == "none"){
        j = 165;
        break;
      }
      if (j == 165 && document.menu12sub.elements[175].value == "none"){
        j = 176;
        break;
      }
      if (j == 176 && document.menu12sub.elements[186].value == "none"){
        j = 187;
        break;
      }
      if (j == 187 && document.menu12sub.elements[197].value == "none"){
        j = 198;
        break;
      }
      if (j == 198 && document.menu12sub.elements[208].value == "none"){
        j = 209;
        break;
      }
      if (j == 209 && document.menu12sub.elements[219].value == "none"){
        j = 222;
        break;
      }
      if (i == 3 || i == 5 || i == 7 || i == 8){
        k_array = str.split;
        if (k_array.length != 2){
          document.menu12sub.elements[j+1].value = "*";
        }
      }
      j++;
    }
    if (i != 0){
      if (cron_ck(array_cron,j) == -1){
         error_ct++;
      }
      else check++;
    }
  }
  if(error_ct == 0 && check > 0){
    document.getElementById("menu12sub_form").submit();
  }
  else return false;
}

function logout(){
  if(window.confirm('Are you sure you want to run the log out ?\r\nTo close the browser when you run')){
   　location.href='./logout.cgi' ;
  　　(window.open('','_top').opener=top).close();
}
  else {
    window.alert('It has been canceled');
    return false;
  }
}

function menu13_ck(){
  var check = 0;
  var error_ct = 0;
  var array_server= new Array(2);
  array_server[0] = document.menu13.server_val_0.value;
  array_server[1] = document.menu13.server_val_1.value;
  var web_user = array_server[0];
  if (web_user.length != 0) check++;
  var web_password = array_server[1];
  if (web_password.length != 0) check++;
  if (user_ck(web_user,web_password) == -1) error_ct++;
  if(error_ct == 0){
    document.getElementById("menu13_form").submit();
  }
  else {
    alert('There is an error in the item or Input content that has not been Input');
  }
}

function menu14_ck(){
  if(!document.menu14.cmd.checked){
     for (var i=0; i < document.menu14.cmd.length; i++){
       if (document.menu14.cmd[i].checked){
         var msg = document.menu14.cmd[i].value;
       }
     }
  }
  if(window.confirm('Do you really want to ' + msg + '?')){
    document.getElementById("menu14_form").submit();
  }
  else {
    window.alert('It has been canceled');
    return false;
  }
}

function menu15_ck(item){
  var YMD = "";
  var HMS = "";
  var Dstr = "";
  var Tstr = "";
  Dstr = document.menu15.server_date.value;
  Tstr = document.menu15.server_time.value;
  if (Tstr.length == 0 || Dstr.length == 0){
    var date = new Date();
    var YY = date.getYear();
    var MM = date.getMonth() + 1;
    var DD = date.getDate();
    var hh = date.getHours();
    var mm = date.getMinutes() + 1;
    var ss = "00";
    if (YY < 2000){ YY += 1900; }
    if (MM < 10){ MM = "0" + MM; }
    if (DD < 10){ DD = "0" + DD; }
    if (hh < 10){ hh = "0" + hh; }
    if (mm < 10){ mm = "0" + mm; }
    if (mm == 60){
      mm = "00";
      hh = hh + 1;
      if (hh > 23){ hh = "00"; }
    }
    YMD = YY + "/" + MM + "/" + DD;
    HSD = hh + ":" + mm + ":" + ss;
    $("#menu15_server_date").html('Date：<INPUT TYPE="text" style="width:70px;text-align:left;"' + '" VALUE="' + YMD + '" ' + 'NAME="server_date" onClick="return menu15_ck()">&nbsp;');
    $("#menu15_server_time").html('Time：<INPUT TYPE="text" style="width:60px;text-align:left;"' + '" VALUE="' + HSD + '" ' + 'NAME="server_time" onClick="return menu15_ck()">');
  }
  else{
    if (item == "Set"){
      if (input_date_ck(Dstr) && input_timer_ck(Tstr)){
        document.getElementById("menu15_form").submit();
      }
    }
    else {
      return false
    }
  }
}
