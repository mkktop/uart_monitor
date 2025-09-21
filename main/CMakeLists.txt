idf_component_register(SRCS "main.c" "app/app_http_serve.c"
                    INCLUDE_DIRS "." "app"
                    EMBED_FILES "html/index.html" 
                    )