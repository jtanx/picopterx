(function($)
{
        $.fn.blink = function(options)
        {
                var defaults = { delay:500 };
                var options = $.extend(defaults, options);
                
                return this.each(function()
                {
                        var obj = $(this);
                        if (obj.attr("timerid") > 0) return;
                        var timerid=setInterval(function()
                        {
                                if($(obj).css("visibility") == "visible")
                                {
                                        $(obj).css('visibility','hidden');
                                }
                                else
                                {
                                        $(obj).css('visibility','visible');
                                }
                        }, options.delay);
                        obj.attr("timerid", timerid);
                });
        }
        $.fn.unblink = function(options) 
        {
                var defaults = { visible:true };
                var options = $.extend(defaults, options);
                
                return this.each(function() 
                {
                        var obj = $(this);
                        if (obj.attr("timerid") > 0) 
                        {
                                clearInterval(obj.attr("timerid"));
                                obj.attr("timerid", 0);
                                obj.css('visibility', options.visible?'visible':'hidden');
                        }
                });
        }
}(jQuery))