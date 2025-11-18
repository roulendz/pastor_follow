from gui.simple_ui import run_app
from raw_tracking import RawSessionLogger

if __name__ == "__main__":
    logger = RawSessionLogger(sLogPath="f:\\Documents\\Arduino\\pastor_follow_v2\\session_info.log")
    try:
        run_app()
    except Exception as e:
        try:
            logger.log_failure(f"app_crash:{e}")
        except Exception:
            pass
        finally:
            try:
                logger.close()
            except Exception:
                pass
        raise