$(function(){function e(){return!!window.navigator.userAgent.toLowerCase().match(/(csdn)/i)||!!l}function t(e){if(void 0!==typeof e&&null!==e&&""!==e)return e=e.toLowerCase(),class_name="hl-1",class_name}function n(e){return void 0===e||null===e||""===e}function o(e,t){n(t)||(t=t.toLowerCase().replace(/(^\s*)|(\s*$)/g,""),n(t)||(t.indexOf(" ")>0?$.each(t.split(/ +/g),function(t,n){o(e,n)}):"..."!==t&&t.length>1&&"&nbsp;"!==t&&"csdn"!==t&&"||"!==t&&(e[t]=0)))}function i(e){var t={};keyword=e.join(" "),o(t,keyword);var n=Object.keys(t);return n.sort(function(e,t){return t.length-e.length}),n}function r(n,o){for(var i=(o.html(),""),r=o.context.childNodes,a=0;a<r.length;a++){var s=r[a];if("#text"===s.nodeName){var l=s.textContent.replace(/</gi,"&lt;"),u=/[\u4e00-\u9fa5]/gi,w=n.map(function(e){return u.test(e)?e:"\\b"+e+"\\b"}),h=new RegExp(w.join("|"),"ig"),p=!1;l=l.replace(h,function(n){var o=n.toLowerCase(),i="https://so.csdn.net/so/search?q="+encodeURIComponent(n)+"&spm=1001.2101.3001.7020";i=e()?"csdnapp://app.csdn.net/search/searchRoot?keyword="+encodeURIComponent(n)+"&t=blog&u="+username:i;var r='{"spm":"1001.2101.3001.7020","dest":"'+i+'"}';return c[o]>=1||d===o||p?n:(p=!0,c[o]=c[o]?c[o]+1:1,d=o,e()?'<a data-href="'+i+'" class="app-hl hl '+t(o)+'" data-report-click='+r+">"+n+"</a>":'<a href="'+i+'" target="_blank" class="hl '+t(o)+'" data-report-view='+r+" data-report-click="+r+" data-tit="+n+" data-pretit="+d+">"+n+"</a>")}),i+=l}else console.log(s.outerHTML),i+=s.outerHTML?s.outerHTML:""}o[0].innerHTML=i,e()||window.csdn.report.viewCheck()}function a(){$("a.app-hl").off("click"),$("a.app-hl").click(function(e){e.preventDefault();var t=$(this).data("href"),n=$(this).attr("data-report-click"),o={trackingInfo:n,action:"app_blog_highlight"},i={url:t};w&&(window.jsCallBackListener.csdntrackevent(JSON.stringify(o)),window.jsCallBackListener.csdnjumpnewpage(JSON.stringify(i))),h&&(window.webkit.messageHandlers.csdntrackevent.postMessage(JSON.stringify(o)),window.webkit.messageHandlers.csdnjumpnewpage.postMessage(JSON.stringify(i)))})}function s(){window.keyword_list_init=!0,$.ajax({type:"GET",url:"https://redisdatarecall.csdn.net/recommend/get_head_word?bid=blog-"+articleId,dataType:"json",timeout:2e3,xhrFields:{withCredentials:!0},success:function(t){if(200==t.status&&t.content){var n=i(t.content),o=t.ext?t.ext:{};if(void 0===n||0===n.length)return;window.keyword_list=n,window.keyword_list_json=o;var s=$("#content_views").find("p,h1,h2,h3,h4,h5,h6");window.keyword_list_doms=s,$.each(s,function(e,t){var t=$(t);r(n,t)}),e()&&a()}}})}var c={},d="",l=!!window.isApp&&window.isApp,u=navigator.userAgent,w=u.indexOf("Android")>-1||u.indexOf("Adr")>-1,h=!!u.match(/\(i[^;]+;( U;)? CPU.+Mac OS X/),p=void 0!==window.showHeadWord&&window.showHeadWord;if(p){s();var f=$("#keywordDecBox");f.length&&($(document).on("mouseover","#article_content .hl",function(){var e=$(this).attr("data-tit"),t=$(this).attr("data-pretit"),n=Object(keyword_list_json)[t];if(e&&n){var o=$(this).offset().left,i=$(this).offset().top,r=$(this).outerHeight(!0),a='<span class="tit">'+e+'</span><span class="dec">'+n+"</span>";f.html(a).css({left:o,top:i+r}).show()}}),$(document).on("mouseout","#article_content .hl",function(){f.hide()}),f.on({mouseover:function(){$(this).show()},mouseout:function(){$(this).hide()}}))}}),function(){function e(){return!!window.navigator.userAgent.toLowerCase().match(/(phone|pad|pod|iphone|ipod|ios|ipad|android|mobile|blackberry|iemobile|mqqbrowser|juc|fennec|wosbrowser|browserng|webos|symbian|windows phone)/i)}function t(){return!!window.navigator.userAgent.toLowerCase().match(/(csdn)/i)}function n(){var n=null;n=e()?document.querySelectorAll('[class^="container-fluid container-fluid-flex container-"]'):t()?document.querySelectorAll('[class^="recommend_item type_"]'):document.querySelectorAll('[class^="recommend-item-box type_"]'),null!==n&&n.length<=5&&$.get("https://statistic.csdn.net/blog/recommend?count="+n.length+"&articleId="+articleId)}n()}();